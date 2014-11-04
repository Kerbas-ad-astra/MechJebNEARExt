using System;
using System.Linq;
using UnityEngine;
using NEAR;
using System.Reflection;

namespace MuMech
{
    public class MechJebNEARExt : ComputerModule
    {

        public MechJebNEARExt(MechJebCore core) : base(core) { }

        bool isFarLoaded = false;

        FieldInfo FieldStall;

        FieldInfo FieldCurrentLift;
        FieldInfo FieldCurrentDrag;

        FieldInfo FieldPitchLocation;
        FieldInfo FieldYawLocation;
        FieldInfo FieldRollLocation;

        private void partModuleUpdate(PartModule pm)
        {
            if (!isFarLoaded || !(pm is FARControllableSurface))
                return;

            if (!(vessel.staticPressure > 0))
                return;

            FARControllableSurface fcs = (FARControllableSurface)pm;

            Vector3d forcePosition = fcs.AerodynamicCenter - vesselState.CoM;
            Vector3 CurWingCentroid = fcs.WingCentroid();
            Vector3d velocity = part.Rigidbody.GetPointVelocity(CurWingCentroid) + Krakensbane.GetFrameVelocity();

            // First we save the curent state of the part

            double stall = fcs.GetStall();
            double cl = fcs.Cl;
            double cd = fcs.Cd;
            double cm = fcs.Cm;

            float currentLift = (float)(FieldCurrentLift.GetValue(fcs));
            float currentDrag = (float)(FieldCurrentDrag.GetValue(fcs));

            double maxdeflect = 0;
            if (fcs.pitchaxis)
            {
                maxdeflect += (double)(FieldPitchLocation.GetValue(fcs));
            }
            if (fcs.yawaxis)
            {
                maxdeflect += (double)(FieldPitchLocation.GetValue(fcs));
            }
            if (fcs.rollaxis)
            {
                maxdeflect += (double)(FieldPitchLocation.GetValue(fcs));
            }

            maxdeflect = maxdeflect.Clamp(-Math.Abs(fcs.maxdeflect), Math.Abs(fcs.maxdeflect));

            // Then we turn it one way
            double AoA = fcs.CalculateAoA(velocity, maxdeflect);
            vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, AoA))));

            // We restore it to the initial state
            //fcs.stall = stall
            FieldStall.SetValue(fcs, stall);
            fcs.Cl = cl;
            fcs.Cd = cd;
            fcs.Cm = cm;

            // And the other way
            AoA = fcs.CalculateAoA(velocity, -maxdeflect);
            vesselState.ctrlTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(Vector3.Cross(forcePosition, fcs.CalculateForces(velocity, AoA))));

            // And in the end we restore its initial state
            //fcs.stall = stall
            FieldStall.SetValue(fcs, stall);
            fcs.Cl = cl;
            fcs.Cd = cd;
            fcs.Cm = cm;

            FieldCurrentLift.SetValue(fcs, currentLift);
            FieldCurrentDrag.SetValue(fcs, currentDrag);
        }

        public override void OnStart(PartModule.StartState state)
        {
            isFarLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "NEAR");

            if (isFarLoaded)
            {
                FieldStall = typeof(FARWingAerodynamicModel).GetField("stall", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

                FieldCurrentLift = typeof(FARWingAerodynamicModel).GetField("currentLift", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
                FieldCurrentDrag = typeof(FARWingAerodynamicModel).GetField("currentDrag", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

                FieldPitchLocation = typeof(FARControllableSurface).GetField("PitchLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
                FieldYawLocation = typeof(FARControllableSurface).GetField("YawLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);
                FieldRollLocation = typeof(FARControllableSurface).GetField("RollLocation", BindingFlags.NonPublic | BindingFlags.GetField | BindingFlags.Instance);

                if (FieldStall != null && FieldCurrentLift != null && FieldCurrentDrag != null &&
                    FieldPitchLocation != null && FieldYawLocation != null && FieldRollLocation != null)
                {
                    print("MechJebNEARExt adding MJ2 callback");
                    vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);
                }
                else
                {
                    isFarLoaded = false;
                    string status = "MechJebNEARExt : failure to initialize reflection calls, a new version may be required";
                    ScreenMessages.PostScreenMessage(status, 10, ScreenMessageStyle.UPPER_CENTER);
                    print(status);
                }
            }
        }
    }
}
