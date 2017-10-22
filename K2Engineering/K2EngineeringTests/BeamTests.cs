using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using K2Engineering;
using Rhino.Geometry;

namespace K2EngineeringTests {
    [TestClass]
    public class BeamTests {
        [TestMethod]
        public void CanCalculateMomentCompA() {
            

            Vector3d vec = new Vector3d(1,0,0);
            Vector3d yax = new Vector3d(0,1,0);
            Vector3d xax = new Vector3d(0,0,1);
            double L = 2;



            double mz = 1000;
            double mx = 1000;
            double my = 1000;


            var comp = BeamGoal.CalculateMomentCompA(vec, mx, yax, my, xax, L);
            //var comp = BeamGoal.CalculateMomentCompB(mz, yax, my, L);

            




        }
    }
}
