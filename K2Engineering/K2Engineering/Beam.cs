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
            List<int> optionals = new List<int>();
            optionals.Add(pManager.AddPlaneParameter("StartFrame", "StartFrame", "The plane at one end of the beam, its Z axis aligned with the beam direction", 0));
            optionals.Add(pManager.AddPlaneParameter("EndFrame", "EndFrame", "Should be parallel to StartFrame", 0));
            optionals.Add(pManager.AddPlaneParameter("StartNode", "StartNode", "The plane defining the node the start of the beam attaches to. If none supplied this defaults to XY aligned", 0));
            optionals.Add(pManager.AddPlaneParameter("EndNode", "EndNode", "The plane defining the node the end of the beam attaches to. If none supplied this defaults to XY aligned", 0));
            pManager.AddNumberParameter("E", "E", "Young's Modulus", 0, 1.0);
            pManager.AddNumberParameter("A", "A", "Cross-section area", 0, 1.0);
            pManager.AddNumberParameter("Ix", "Ix", "2nd moment of area about X", 0, 1.0);
            pManager.AddNumberParameter("Iy", "Iy", "2nd moment of area about Y", 0, 1.0);
            pManager.AddNumberParameter("GJ", "GJ", "Shear modulus * torsional constant", 0, 1.0);

            foreach (int i in optionals) pManager[i].Optional = true;

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager) {
            pManager.AddGenericParameter("B", "Beam", "Beam element with force and stress output", GH_ParamAccess.item);
            

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA) {
            Plane startPlane = default(Plane);
            Plane endPlane = default(Plane);
            Plane startNode = default(Plane);
            Plane endNode = default(Plane);
            double e = 0.0;
            if (DA.GetData<Plane>(0, ref startPlane) && DA.GetData<Plane>(1, ref endPlane) && DA.GetData<double>(4, ref e)) {
                if (!DA.GetData<Plane>(2, ref startNode)) {
                    startNode = new Plane(startPlane.Origin, Vector3d.XAxis, Vector3d.YAxis);
                }
                if (!DA.GetData<Plane>(3, ref endNode)) {
                    endNode = new Plane(endPlane.Origin, Vector3d.XAxis, Vector3d.YAxis);
                }
                double l = startPlane.Origin.DistanceTo(endPlane.Origin);
                DA.SetData(0, new BeamGoal(startPlane, endPlane, startNode, endNode, l, e, 1.0, 1.0, 1.0, 1.0));
            }
        }

        public override Guid ComponentGuid {
            get { return new Guid("F548AE24-9567-4A2A-8252-D79A5BBEA5F7"); }
        }
        
    }

    public class BeamGoal : GoalObject {
        public Plane P0;
        public Plane P1;
        public Plane P0R;
        public Plane P1R;
        public double E;
        public double A;
        public double Ix;
        public double Iy;
        public double GJ;
        public double RestLength;
        public double TX1;
        public double TX2;
        public double TY1;
        public double TY2;
        public double twist;

        public BeamGoal(Plane StartPlane, Plane EndPlane, Plane StartNode, Plane EndNode, double L, double E, double A, double Ix, double Iy, double GJ) {
            this.P0 = StartPlane;
            this.P1 = EndPlane;
            this.P0.Transform(Transform.ChangeBasis(Plane.WorldXY, StartNode));
            this.P1.Transform(Transform.ChangeBasis(Plane.WorldXY, EndNode));
            this.RestLength = L;
            this.E = E;
            this.A = A;
            this.Ix = Ix;
            this.Iy = Iy;
            this.GJ = GJ;
            base.PPos = new Point3d[]
			{
				StartPlane.Origin,
				EndPlane.Origin
			};
            base.Move = new Vector3d[2];
            base.Weighting = new double[]
			{
				E,
				E
			};
            base.Torque = new Vector3d[2];
            base.TorqueWeighting = new double[]
			{
				E,
				E
			};
            base.InitialOrientation = new Plane[]
			{
				StartNode,
				EndNode
			};
            this.P0R = StartPlane;
            this.P1R = EndPlane;
        }
        public override void Calculate(List<KangarooSolver.Particle> p) {
            //get the current positions/orientations of the nodes
            Plane orientation = p[base.PIndex[0]].Orientation;
            Plane orientation2 = p[base.PIndex[1]].Orientation;

            //get the initial orientations of the beam end frames..
            this.P0R = this.P0;
            this.P1R = this.P1;
            //..and transform them to get the current beam end frames
            this.P0R.Transform(Transform.PlaneToPlane(Plane.WorldXY, orientation));
            this.P1R.Transform(Transform.PlaneToPlane(Plane.WorldXY, orientation2));

            //axial (ignoring elongation due to bowing for now)
            Vector3d current = this.P1R.Origin - this.P0R.Origin;
            double length = current.Length;
            double num = length - this.RestLength;
            Vector3d axialMove = 0.5 * (current / length) * num;
            Vector3d xAxis = this.P0R.XAxis;
            Vector3d yAxis = this.P0R.YAxis;
            Vector3d xAxis2 = this.P1R.XAxis;
            Vector3d yAxis2 = this.P1R.YAxis;
            Vector3d vector3d3 = current;
            vector3d3.Unitize();

            //bend angles
            this.TX1 = yAxis * vector3d3;
            this.TX2 = yAxis2 * vector3d3;
            this.TY1 = xAxis * vector3d3;
            this.TY2 = xAxis2 * vector3d3;

            //twist
            this.twist = (xAxis * yAxis2 - xAxis2 * yAxis) / 2.0;

            //moments
            Vector3d moment1 = xAxis * this.TX1 - yAxis * this.TY1;
            Vector3d moment2 = xAxis2 * this.TX2 - yAxis2 * this.TY2;

            base.Torque[0] = -0.25 * (moment1 + this.twist * current);
            base.Torque[1] = -0.25 * (moment2 - this.twist * current);
            base.TorqueWeighting[0] = (base.TorqueWeighting[1] = this.E * this.A);

            //  shears
            Vector3d SY1 = 0.25 * Vector3d.CrossProduct(this.TX1 * xAxis, current);
            Vector3d SX1 = 0.25 * Vector3d.CrossProduct(this.TY1 * yAxis, current);
            Vector3d SY2 = 0.25 * Vector3d.CrossProduct(this.TX2 * xAxis2, current);
            Vector3d SX2 = 0.25 * Vector3d.CrossProduct(this.TY2 * yAxis2, current);

            base.Move[0] = axialMove + SX1 - SY1 + SX2 - SY2;
            base.Move[1] = -base.Move[0];
        }


        public override object Output(List<KangarooSolver.Particle> p) {
            return new List<object>
			{
				this.P0R,
				this.P1R,
				this.TX1,
				this.TX2,
				this.TY1,
				this.TY2,
				this.twist
			};
        }
    }
    /*
    public class BeamGoal : GoalObject {

        public Plane P0; //Original plane at first node
        public Plane P1; //Original plane at second node
        public Plane P0R; //Current plane at first node
        public Plane P1R; //Current plane at second node

        public double E; //YoungsModulus;
        public double Ix; //SecondMomentOfAreaAroundXX
        public double Iy; //SecondMomentOfAreaAroundYY
        public double G; // ShearModulus;
        public double J; // MomentOfInertiaInTorsion
        private double _scale;


        private Vector3d F1, F2, M1, M2;


        public double A, GJ, L0;
        public double TX1, TX2, TY1, TY2, twist;


        
        public BeamGoal() {
            _scale = 1;
        }

        public BeamGoal(Plane StartPlane, Plane EndPlane, Plane StartNode, Plane EndNode,
          double E, double A, double Ix, double Iy, double G, double J) :this() {
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

            PPos = new Point3d[2] { StartPlane.Origin, EndPlane.Origin };
            Move = new Vector3d[2];
            Weighting = new double[2] { 1 / _scale, 1 / _scale };
            Torque = new Vector3d[2];
            TorqueWeighting = new double[2] { 1 / _scale, 1 / _scale };

            InitialOrientation = new Plane[2] { StartNode, EndNode };

            P0R = StartPlane;
            P1R = EndPlane;
        }

        public double CalculateTheta(Vector3d p, Vector3d x) {
            p.Unitize();
            return p * x;
        }

        public double CalculateTwist(Vector3d x1, Vector3d x2, Vector3d y1, Vector3d y2) {
            return (x1 * y2 - x2 * y1) * 0.5;
        }

        public double CalculateElongation(Vector3d p, double L0, double Tx1, double Tx2, double Ty1, double Ty2) {
            return (p * p - L0 * L0) / (2 * L0) + L0 / 60 * (4 * (Tx1 * Tx1 + Ty1 * Ty1) - 2 * (Tx1 * Tx2 - Ty1 * Ty2) + 4 * (Tx2 * Tx2 + Ty2 * Ty2));
        }

        public double CalculateN(double E, double A, double L0, double e) {
            return E * A / L0 * e;
        }

        public double CalculateM(double N, double L0, double Tx1, double Tx2) {
            return N * L0 / 30 * (4 * Tx1 - Tx2) + 2 * E * Ix / L0 * (2 * Tx1 + Tx2);
        }

        public double CalculateMT(double G, double J, double L0, double Tz) {
            return G * J * Tz / L0;
        }

        public Vector3d CalculateForceAtNode1(double N, Vector3d p, double Mx1, Vector3d y1, double My1, Vector3d x1,
          double Mx2, Vector3d y2, double My2, Vector3d x2, double L0) {
            return (N * p + Mx1 * y1 - My1 * x1 + Mx2 * y2 + My2 * x2) / L0;
        }

        private Vector3d CalculateMomentCompA(Vector3d p, double Mx, Vector3d y, double My, Vector3d x, double L0) {
            return Mx * Vector3d.CrossProduct(y, p) / L0 - My * Vector3d.CrossProduct(x, p) / L0;
        }

        private Vector3d CalculateMomentCompB(double MT, Vector3d y1, Vector3d x1, Vector3d y2, Vector3d x2) {
            return MT * (Vector3d.CrossProduct(x1, y2) - Vector3d.CrossProduct(y1, x2));
        }

        private double N;
        private double Mx1;
        private double Mx2;
        private double My1;
        private double My2;
        private double Mz;

        public override void Calculate(List<KangarooSolver.Particle> p) {
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

           /////////////////////////////////////////////////////////////////////////////////////////


            double CurrentLength = Current.Length;
            double Stretch = CurrentLength - L0;
            Vector3d AxialMove = 0.5 * (Current / CurrentLength) * Stretch;

            Vector3d X1 = P0R.XAxis;
            Vector3d Y1 = P0R.YAxis;
            Vector3d X2 = P1R.XAxis;
            Vector3d Y2 = P1R.YAxis;

            //bend angles
            Vector3d UnitCurrent = Current;
            UnitCurrent.Unitize();
            TX1 = Y1 * UnitCurrent;
            TX2 = Y2 * UnitCurrent;
            TY1 = X1 * UnitCurrent;
            TY2 = X2 * UnitCurrent;

            //twist
            twist = ((X1 * Y2) - (X2 * Y1)) / 2.0;

            //moments
            Vector3d Moment1 = (X1 * TX1) - (Y1 * TY1);
            Vector3d Moment2 = (X2 * TX2) - (Y2 * TY2);

            Torque[0] = -0.25 * (Moment1 + twist * Current);
            Torque[1] = -0.25 * (Moment2 - twist * Current);
            TorqueWeighting[0] = TorqueWeighting[1] = E * A;

            //  shears
            Vector3d SY1 = 0.25 * Vector3d.CrossProduct(TX1 * X1, Current);
            Vector3d SX1 = 0.25 * Vector3d.CrossProduct(TY1 * Y1, Current);
            Vector3d SY2 = 0.25 * Vector3d.CrossProduct(TX2 * X2, Current);
            Vector3d SX2 = 0.25 * Vector3d.CrossProduct(TY2 * Y2, Current);

            Move[0] = AxialMove + SX1 - SY1 + SX2 - SY2;
            Move[1] = -Move[0];



            //////////////////////////////////////////////////////////////////////////////////////////////



            //Determine member forces
            double e = CalculateElongation(Current, L0, Tx1, Tx2, Ty1, Ty2);
            N = CalculateN(E, A, L0, e);
            Mx1 = CalculateM(N, L0, Tx1, Tx2);
            Mx2 = CalculateM(N, L0, Tx2, Tx1);
            My1 = CalculateM(N, L0, Ty1, Ty2);
            My2 = CalculateM(N, L0, Ty2, Ty1);
            Mz = CalculateMT(G, J, L0, Tz);

            //Determine end point forces
            //F1 = CalculateForceAtNode1(N, Current, Mx1, y1, My1, x1, Mx2, y2, My2, x2, L0);
            //F2 = -1 * F1;

            //Vector3d MCompB = CalculateMomentCompB(Mz, y1, x1, y2, x2);
            //M1 = -(CalculateMomentCompA(Current, Mx1, y1, My1, x1, L0) + MCompB);
            //M2 = -(CalculateMomentCompA(Current, Mx2, y2, My2, x2, L0) + MCompB);

            //Move
            //Move[0] = F1 * _scale;
            //Move[1] = F2 * _scale;
            //Torque[0] = M1 * _scale;
            //Torque[1] = M2 * _scale;
            //Move[0] = AxialMove + SX1 - SY1 + SX2 - SY;
            //Move[1] = -Move[0];

            //Weighting
            //Weighting[0] = F1.Length;
            //Weighting[1] = F2.Length;
            //TorqueWeighting[0] = M1.Length;
            //TorqueWeighting[1] = M2.Length;
        }
        public override object Output(List<KangarooSolver.Particle> p) {
            List<object> DataOut = new List<object>();

            DataOut.Add(P0R);
            DataOut.Add(P1R);
            DataOut.Add(N);
            DataOut.Add(Mz);
            DataOut.Add(Mx1);
            DataOut.Add(My1);
            DataOut.Add(Mx2);
            DataOut.Add(My2);

            return DataOut;
        }
    }
    */
}

