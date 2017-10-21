using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace K2Engineering {
    public class Beam {

        public Beam() {
        
        }

        public void Calculate() {
            
            Plane P0; //Original plane at first node
            Plane P1; //Original plane at second node
            Plane P0R; //Current plane at first node
            Plane P1R; //Current plane at second node

            double E; //YoungsModulus;
            double Ix; //SecondMomentOfAreaAroundXX
            double Iy;//SecondMomentOfAreaAroundYY
            double G; // ShearModulus;
            double J; // MomentOfInertiaInTorsion
            double iL; //InitialLength 

            //Calculate normal force


        }

        public double CalculateThetaX1() {
            return -1;
        }

        public double CalculateThetaX2() {
            return -1;
        }

        public double CalculateThetaY1() {
            return -1;
        }

        public double CalculateThetaY2() {
            return -1;
        }

        public double CalculateTwist() {
            return -1;
        }

        public double CalculateMx1() {
            return -1;
        }

        public double CalculateMx2() {
            return -1;
        }

        public double CalculateMy1() {
            return -1;
        }

        public double CalculateMy2() {
            return -1;
        }

        public double CalculateTorsion() {
            return -1;
        }

        public double CalculateNormalForce() {
            return -1;
        }






    }
}

