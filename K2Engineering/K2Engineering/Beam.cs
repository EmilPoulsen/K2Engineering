using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KangarooSolver;
using Grasshopper.Kernel;

namespace K2Engineering {

    public class Beam : GH_Component {
        /// <summary>
        /// Initializes a new instance of the Bar class.
        /// </summary>
        public Beam()
            : base("Beam", "Beam",
                "A goal that represents a beam element.",
                "K2Eng", "0 Elements") {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager) {
            pManager.AddPlaneParameter("StartPlane", "SPlane", "Start Plane", GH_ParamAccess.item);
            pManager.AddPlaneParameter("EndPlane", "EPlane", "End Plane", GH_ParamAccess.item);
            pManager.AddNumberParameter("E", "E", "E", GH_ParamAccess.item);
            pManager.AddNumberParameter("A", "A", "A", GH_ParamAccess.item);
            pManager.AddNumberParameter("Ix", "Ix", "Ix", GH_ParamAccess.item);
            pManager.AddNumberParameter("Iy", "Iy", "Iy", GH_ParamAccess.item);
            pManager.AddNumberParameter("G", "G", "G", GH_ParamAccess.item);
            pManager.AddNumberParameter("J", "J", "J", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager) {
            pManager.AddGenericParameter("B", "Bar", "Bar element with force and stress output", GH_ParamAccess.item);
            

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA) {
            //Input
            Plane startPlane = Plane.Unset;
            Plane endPlane = Plane.Unset;
            double E = 0;
            double A = 0;
            double L = 0;
            double Ix = 0;
            double Iy = 0;
            double G = 0;
            double J = 0;

            if (!DA.GetData(0, ref startPlane)) return;
            if (!DA.GetData(1, ref endPlane)) return;
            if (!DA.GetData(2, ref E)) return;
            if (!DA.GetData(3, ref A)) return;
            if (!DA.GetData(4, ref Ix)) return;
            if (!DA.GetData(5, ref Iy)) return;
            if (!DA.GetData(6, ref G)) return;
            if (!DA.GetData(7, ref J)) return;
           
            //Create instance of bar
            //GoalObject barElement = new BeamGoal(startPlane, endPlane, L, E, A, Ix, Iy, GJ);
            GoalObject beamElement = new BeamGoal(startPlane, endPlane, startPlane, endPlane, E, A, Ix, Iy, G, J);

            //Output
            DA.SetData(0, beamElement);
        }

        public override Guid ComponentGuid {
            get { return new Guid("F548AE24-9567-4A2A-8252-D79A5BBEA5F7"); }
        }
        
    }


    public class BeamGoal : GoalObject
    {

        public Plane P0; //Original plane at first node
        public Plane P1; //Original plane at second node
        public Plane P0R; //Current plane at first node
        public Plane P1R; //Current plane at second node

        public double E; //YoungsModulus;
        public double Ix; //SecondMomentOfAreaAroundXX
        public double Iy; //SecondMomentOfAreaAroundYY
        public double G; // ShearModulus;
        public double J; // MomentOfInertiaInTorsion

        private Vector3d F1, F2, M1, M2;


        public double A, GJ, L0;
        public double TX1, TX2, TY1, TY2, twist;

        public BeamGoal(Plane StartPlane, Plane EndPlane, Plane StartNode, Plane EndNode, 
            double E, double A, double Ix, double Iy, double G, double J)

        {
            this.P0 = StartPlane;
            this.P1 = EndPlane;

            this.P0.Transform(Transform.ChangeBasis(Plane.WorldXY, StartNode));
            this.P1.Transform(Transform.ChangeBasis(Plane.WorldXY, EndNode));

            L0 = StartPlane.Origin.DistanceTo(EndPlane.Origin);
            this.E = E;
            this.A = A;
            this.Ix = Ix;
            this.Iy = Iy;
            this.GJ = G * J;


            PPos = new Point3d[2] {StartPlane.Origin, EndPlane.Origin};
            Move = new Vector3d[2];
            Weighting = new double[2] {1, 1};
            Torque = new Vector3d[2];
            TorqueWeighting = new double[2] {1, 1};

            InitialOrientation = new Plane[2] {StartNode, EndNode};

            P0R = StartPlane;
            P1R = EndPlane;
        }

        
        public double CalculateTheta(Vector3d p, Vector3d x)
        {
            p.Unitize();
            return p*x;
        }

        public double CalculateTwist(Vector3d x1, Vector3d x2, Vector3d y1, Vector3d y2)
        {
            return (x1*y2 - x2*y1) * 0.5;
        }

        public double CalculateElongation(Vector3d p, double L0, double Tx1, double Tx2, double Ty1, double Ty2)
        {
            return (p*p - L0*L0)/(2*L0) + L0/60*(4*(Tx1*Tx1 + Ty1*Ty1) - 2*(Tx1*Tx2 - Ty1*Ty2) + 4*(Tx2*Tx2 + Ty2*Ty2));
        }

        public double CalculateN(double E, double A, double L0, double e)
        {
            return E*A/L0*e;
        }

        public double CalculateM(double N, double L0, double Tx1, double Tx2)
        {
            return N*L0/30*(4*Tx1 - Tx2) + 2*E*Ix/L0*(2*Tx1 + Tx2);
        }

        public double CalculateMT(double G, double J, double L0, double Tz)
        {
            return G * J * Tz / L0;
        }

        public Vector3d CalculateForceAtNode1(double N, Vector3d p, double Mx1, Vector3d y1, double My1, Vector3d x1,
            double Mx2, Vector3d y2, double My2, Vector3d x2, double L0)
        {
            return (N*p + Mx1*y1 - My1*x1 + Mx2*y2 + My2*x2)/L0;
        }

        private Vector3d CalculateMomentCompA(Vector3d p, double Mx, Vector3d y, double My, Vector3d x, double L0)
        {
            return Mx*Vector3d.CrossProduct(y, p)/L0 - My*Vector3d.CrossProduct(x, p)/L0;
        }

        private Vector3d CalculateMomentCompB(double MT, Vector3d y1, Vector3d x1, Vector3d y2, Vector3d x2)
        {
            return MT*(Vector3d.CrossProduct(x1, y2) - Vector3d.CrossProduct(y1, x2));
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
        	  //get the current positions/orientations of the nodes
            Plane NodeACurrent = p[PIndex[0]].Orientation;
            Plane NodeBCurrent = p[PIndex[1]].Orientation;

            //get the initial orientations of the beam end frames..
            P0R = P0;
            P1R = P1;
            //..and transform them to get the current beam end frames
            P0R.Transform(Transform.PlaneToPlane(Plane.WorldXY, NodeACurrent));
            P1R.Transform(Transform.PlaneToPlane(Plane.WorldXY, NodeBCurrent));
            
            //Find local angle changes
            Vector3d x1 = P0R.XAxis;
            Vector3d x2 = P1R.XAxis;
            Vector3d y1 = P0R.YAxis;
            Vector3d y2 = P1R.YAxis;
            
            Vector3d Current = P1R.Origin - P0R.Origin;
            double Ty1 = CalculateTheta(Current, x1);
            double Tx1 = CalculateTheta(Current, y1);
            double Ty2 = CalculateTheta(Current, x2);
            double Tx2 = CalculateTheta(Current, y2);
            double Tz = CalculateTwist(x1, x2, y1, y2);
            
            //Determine member forces
            double e = CalculateElongation(Current, L0, Tx1, Tx2, Ty1, Ty2);
            double N = CalculateN(E, A, L0, e);
            double Mx1 = CalculateM(N, L0, Tx1, Tx2);
            double Mx2 = CalculateM(N, L0, Tx2, Tx1);
            double My1 = CalculateM(N, L0, Ty1, Ty2);
            double My2 = CalculateM(N, L0, Ty2, Ty1);
            double Mz = CalculateMT(G, J, L0, Tz);
            
            //Determine end point forces
            F1 = CalculateForceAtNode1(N, Current, Mx1, y1, My1, x1, Mx2, y2, My2, x2, L0);
            F2 = -1 * F1;
                        
            Vector3d MCompB = CalculateMomentCompB(Mz, y1, x1, y2, x2);
            M1 = - (CalculateMomentCompA(Current, Mx1, y1, My1, x1, L0) + MCompB);
            M2 = - (CalculateMomentCompA(Current, Mx2, y2, My2, x2, L0) + MCompB);
            
            //Move
            Move[0] = F1;
            Move[1] = F2;
            Torque[0] = M1;
            Torque[1] = M2;
          }
        public override object Output(List<KangarooSolver.Particle> p)
        {
            List<object> DataOut = new List<object>();

            DataOut.Add(F1);
            DataOut.Add(F2);
            DataOut.Add(M1);
            DataOut.Add(M2);
            DataOut.Add(this.P0R);
            DataOut.Add(this.P1R);

            return DataOut;
        }
    }
}

