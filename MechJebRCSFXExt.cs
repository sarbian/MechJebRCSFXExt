using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using MuMech;
using UnityEngine;


namespace MechJebRCSFXExt
{
    public class MechJebRCSFXExt : ComputerModule
    {
        public MechJebRCSFXExt(MechJebCore core) : base(core) { }

        private void partModuleUpdate(PartModule pm)
        {
            if (!(pm is ModuleRCSFX))
                return;

            //MechJebCore.print("ModuleRCSFX");

            ModuleRCSFX rcs = (ModuleRCSFX)pm;

            if (!rcs.rcsEnabled || part.ShieldedFromAirstream || !vessel.ActionGroups[KSPActionGroup.RCS])
                return;

            //Vector3 CoM = vesselState.CoM + vessel.rb_velocity * Time.fixedDeltaTime;
            Vector3 CoM = vessel.CoM + vessel.rb_velocity * Time.fixedDeltaTime;

            //Vector3 inputAngular = vessel.ReferenceTransform.rotation * new Vector3(rcs.enablePitch ? 1f : 0f, rcs.enableRoll ? 1f : 0f, rcs.enableYaw ? 1f : 0);
            //Vector3 inputLinear  = vessel.ReferenceTransform.rotation * new Vector3(rcs.enableX ? 1f : 0f, rcs.enableZ ? 1f : 0f, rcs.enableY ? 1f : 0f);

            Vector3 inputAngular = vessel.ReferenceTransform.rotation * Vector3.one;
            Vector3 inputLinear = vessel.ReferenceTransform.rotation * Vector3.one;

            int xformCount = rcs.thrusterTransforms.Count;
            bool success; // Just to pass to rcs.CalculateThrust(). Probably not going to use it. Or might!

            float thrustBase = rcs.CalculateThrust(1f, out success);

            for (int i = 0; i < xformCount; ++i)
            {
                Transform xform = rcs.thrusterTransforms[i];
                if (xform.position != Vector3.zero)
                {
                    Vector3 position = xform.position;
                    Vector3 relPos = position - CoM;
                    Vector3 torque = Vector3.Cross(inputAngular.normalized, relPos.normalized);

                    Vector3 thruster = rcs.useZaxis ? xform.forward : xform.up;
                    float leverDistance = rcs.GetLeverDistance(-thruster, vessel.CoM);


                    float thrust = Mathf.Max(Vector3.Dot(thruster, torque), 0f);
                    thrust += Mathf.Max(Vector3.Dot(thruster, inputLinear.normalized), 0f);

                    //if (thrust <= 0f)
                    //{
                    //    print("Ignore " + thrust.ToString("F2") + " " + Vector3.Dot(thruster, torque).ToString("F2") + " " + Vector3.Dot(thruster, inputLinear.normalized).ToString("F2"));
                    //    continue;
                    //}

                    if (FlightInputHandler.fetch.precisionMode)
                    {
                        if (rcs.useLever)
                        {

                            if (leverDistance > 1)
                            {
                                thrust /= leverDistance;
                            }
                        }
                        else
                        {
                            thrust *= rcs.precisionFactor;
                        }
                    }

                    float thrustForce = thrustBase * thrust;

                    Vector3 force = vessel.GetTransform().InverseTransformDirection(thrustForce * thruster);

                    force.Scale(new Vector3(rcs.enableX ? 1f : 0f, rcs.enableZ ? 1f : 0f, rcs.enableY ? 1f : 0f));

                    vesselState.rcsThrustAvailable.Add(force);

                    //Vector3d thrusterTorque = Vector3.Cross(relPos, -thruster) * leverDistance * thrustBase;
                    Vector3d thrusterTorque = thruster * (leverDistance * thrustForce);
                    // Convert in vessel local coordinate

                    thruster.Scale(new Vector3d(rcs.enablePitch ? 1f : 0f, rcs.enableRoll ? 1f : 0f, rcs.enableYaw ? 1f : 0));
                    vesselState.rcsTorqueAvailable.Add(vessel.GetTransform().InverseTransformDirection(thrusterTorque));
                }
            }
        }

        public override void OnStart(PartModule.StartState state)
        {
            bool isModuleRCSFXLoaded = AssemblyLoader.loadedAssemblies.Any(a => a.assembly.GetName().Name == "ModuleRCSFX");

            if (!isModuleRCSFXLoaded)
                return;

            print("[MechJebRCSFXExt] Adding callback");
            //vesselState.vesselStatePartExtensions.Add(partUpdate);
            vesselState.vesselStatePartModuleExtensions.Add(partModuleUpdate);
        }
    }
}
