using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KangarooSolver;

namespace K2Engineering {
    public class Beam : GoalObject
    {

        public Plane P0; //Original plane at first node
        public Plane P1; //Original plane at second node
        public Plane P0R; //Current plane at first node
        public Plane P1R; //Current plane at second node

        public double E; //YoungsModulus;
        public double Ix; //SecondMomentOfAreaAroundXX
        public double Iy;//SecondMomentOfAreaAroundYY
        public double G; // ShearModulus;
        public double J; // MomentOfInertiaInTorsion
        public double iL; //InitialLength 

        public double A, GJ, RestLength;
        public double TX1, TX2, TY1, TY2, twist;


        public Beam(Plane StartPlane, Plane EndPlane, Plane StartNode, Plane EndNode, double L, double E, double A, double Ix, double Iy, double GJ)
        {
            this.P0 = StartPlane;
            this.P1 = EndPlane;

            this.P0.Transform(Transform.ChangeBasis(Plane.WorldXY, StartNode));
            this.P1.Transform(Transform.ChangeBasis(Plane.WorldXY, EndNode));

            RestLength = L;
            this.E = E;
            this.A = A;
            this.Ix = Ix;
            this.Iy = Iy;
            this.GJ = GJ;

            PPos = new Point3d[2] { StartPlane.Origin, EndPlane.Origin };
            Move = new Vector3d[2];
            Weighting = new double[2] { E, E };
            Torque = new Vector3d[2];
            TorqueWeighting = new double[2] { E, E };

            InitialOrientation = new Plane[2] { StartNode, EndNode };

            P0R = StartPlane;
            P1R = EndPlane;
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

            //axial (ignoring elongation due to bowing for now)
            Vector3d Current = P1R.Origin - P0R.Origin;
            double CurrentLength = Current.Length;
            double Stretch = CurrentLength - RestLength;
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
        }

        public override object Output(List<KangarooSolver.Particle> p)
        {
            List<object> DataOut = new List<object>();

            DataOut.Add(P0R);
            DataOut.Add(P1R);
            DataOut.Add(TX1);
            DataOut.Add(TX2);
            DataOut.Add(TY1);
            DataOut.Add(TY2);
            DataOut.Add(twist);

            return DataOut;
        }

    
    public double CalculateThetaX(Vector3d p, Vector3d y)
    {
        p.Unitize();
      return p * y;
    }
    
    public double CalculateThetaY(Vector3d p, Vector3d x)
    {
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
    
    public double CalculateMx(double N, double L0, double Tx1, double Tx2) {
      return N * L0/30 * (4 * Tx1 - Tx2) + 2 * E * Ix / L0 * (2 * Tx1 + Tx2);
    }
    
    public double CalculateTorsion(double GJ, double L0, double psi) {
      return GJ /psi * L0;
    }






    }
}

