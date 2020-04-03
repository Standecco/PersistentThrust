using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace PersistentThrust
{
    public class PersistentEngine : PartModule
    {
        // Flag to activate force if it isn't to allow overriding stage activation
        [KSPField(isPersistant = true)]
        bool IsForceActivated;
        // Flag if using PersistentEngine features
        public bool IsPersistentEngine = false;
        // Flag whether to request massless resources
        public bool RequestPropMassless = false;
        // Flag whether to request resources with mass
        public bool RequestPropMass = true;
        // Flag if the engine is selected when a MultiModeEngine module is present
        public bool IsMultiMode = false;

        // GUI
        // Enable/disable persistent engine features
        [KSPField(isPersistant = true, guiActive = true, guiActiveEditor = true, guiName = "Persistent"), UI_Toggle(disabledText = "Disabled", enabledText = "Enabled")]
        public bool PersistentEnabled = false;
        // Thrust
        [KSPField(guiActive = true, guiName = "Thrust")]
        protected string Thrust = "";
        // Isp
        [KSPField(guiActive = true, guiName = "Isp")]
        protected string Isp = "";
        // Throttle
        [KSPField(guiActive = true, guiName = "Throttle")]
        protected string Throttle = "";

        // Engine module on the same part
        public ModuleEngines engine;
        public MultiModeEngine multiMode;

        // Persistent values to use during timewarp
        public float IspPersistent = 0;
        public float ThrustPersistent = 0;
        public float ThrottlePersistent = 0;

        // Keep track of number of physics ticks skipped
        public int skipCounter = 0;

        // Are we transitioning from timewarp to reatime?
        bool warpToReal = false;

        // Propellant data
        public List<PersistentPropellant> pplist;
        // Average density of propellants
        public double densityAverage;

        /// Initialization
        public override void OnLoad(ConfigNode node)
        {
            // Run base OnLoad method
            base.OnLoad(node);

            // Populate engine and engineFX fields
            FindModuleEngines();

            if (IsPersistentEngine)
            {
                // Initialize PersistentPropellant list
                pplist = PersistentPropellant.MakeList(engine.propellants);

                // Initialize density of propellant used in deltaV and mass calculations
                densityAverage = pplist.AverageDensity();
            }
        }

        /// Update
        public override void OnUpdate()
        {
            if (IsPersistentEngine && PersistentEnabled)
            {
                // When transitioning from timewarp to real update throttle
                if (warpToReal)
                {
                    vessel.ctrlState.mainThrottle = ThrottlePersistent;
                    warpToReal = false;
                }

                // Persistent thrust GUI
                Fields["Thrust"].guiActive = isEnabled;
                Fields["Isp"].guiActive = isEnabled;
                Fields["Throttle"].guiActive = isEnabled;

                // Update display values
                Thrust = Utils.FormatThrust(ThrustPersistent);
                Isp = Math.Round(IspPersistent, 2).ToString() + " s";
                Throttle = Math.Round(ThrottlePersistent * 100).ToString() + "%";

                // Activate force if engine is enabled and operational
                if (!IsForceActivated && engine.isEnabled && engine.isOperational)
                {
                    IsForceActivated = true;
                    part.force_activate();
                }
            }
        }

        /// Physics update
        public override void OnFixedUpdate()
        {
            // Checks if engine mode wasn't switched
            if (IsMultiMode)
            {
                FetchActiveMode();
            }

            if (IsPersistentEngine && FlightGlobals.fetch != null && isEnabled && PersistentEnabled)
            {
                // Time step size
                var dT = TimeWarp.fixedDeltaTime;

                // Realtime mode
                if (!this.vessel.packed)
                {
                    // Update persistent thrust parameters if NOT transitioning from warp to realtime
                    if (!warpToReal)
                        UpdatePersistentParameters();
                }

                // Timewarp mode: perturb orbit using thrust
                else if (part.vessel.situation != Vessel.Situations.SUB_ORBITAL)
                {
                    warpToReal = true; // Set to true for transition to realtime
                    var UT = Planetarium.GetUniversalTime(); // Universal time
                    var m0 = this.vessel.GetTotalMass(); // Current mass
                    var thrustUV = this.part.transform.up; // Thrust direction unit vector
                                                           // Calculate deltaV vector & resource demand from propellants with mass
                    double demandMass;
                    var deltaVV = CalculateDeltaVV(m0, dT, ThrustPersistent, IspPersistent, thrustUV, out demandMass);
                    // Calculate resource demands
                    var demands = CalculateDemands(demandMass);
                    // Apply resource demands & test for resource depletion
                    var depleted = false;
                    var demandsOut = ApplyDemands(demands, ref depleted);
                    // Apply deltaV vector at UT & dT to orbit if resources not depleted
                    if (!depleted)
                    {
                        vessel.orbit.Perturb(deltaVV, UT);
                    }
                    // Otherwise log warning and drop out of timewarp if throttle on & depleted
                    else if (ThrottlePersistent > 0)
                    {
                        Debug.Log("[PersistentThrust] Thrust warp stopped - propellant depleted");
                        ScreenMessages.PostScreenMessage("Thrust warp stopped - propellant depleted", 5.0f, ScreenMessageStyle.UPPER_CENTER);
                        // Return to realtime
                        TimeWarp.SetRate(0, true);
                    }
                }
                // Otherwise, if suborbital, set throttle to 0 and show error message
                // TODO fix persistent thrust orbit perturbation on suborbital trajectory
                else if (vessel.ctrlState.mainThrottle > 0)
                {
                    vessel.ctrlState.mainThrottle = 0;
                    ScreenMessages.PostScreenMessage("Cannot accelerate and timewarp durring sub orbital spaceflight!", 5.0f, ScreenMessageStyle.UPPER_CENTER);
                }
            }
        }

        /// Calculate demands of each resource
        public virtual double[] CalculateDemands(double demandMass)
        {
            var demands = new double[pplist.Count];
            if (demandMass > 0)
            {
                // Per propellant demand
                for (var i = 0; i < pplist.Count; i++)
                {
                    demands[i] = pplist[i].Demand(demandMass);
                }
            }
            return demands;
        }

        /// Apply demanded resources & return results
        /// Updated depleted boolean flag if resource request failed
        public virtual double[] ApplyDemands(double[] demands, ref bool depleted)
        {
            var demandsOut = new double[pplist.Count];
            for (var i = 0; i < pplist.Count; i++)
            {
                var pp = pplist[i];
                // Request resources if:
                // - resource has mass & request mass flag true
                // - resource massless & request massless flag true
                if (((pp.density > 0 && RequestPropMass) || (pp.density == 0 && RequestPropMassless)) && !CheatOptions.InfinitePropellant)
                {
                    var demandOut = part.RequestResource(pp.propellant.name, demands[i]);
                    demandsOut[i] = demandOut;
                    // Test if resource depleted
                    // TODO test if resource partially depleted: demandOut < demands[i]
                    // For the moment, just let the full deltaV for time segment dT be applied
                    if (demandOut == 0)
                    {
                        depleted = true;
                        Debug.Log(String.Format("[PersistentThrust] Part {0} failed to request {1} {2}", part.name, demands[i], pp.propellant.name));
                    }
                }
                // Otherwise demand is 0
                else
                {
                    demandsOut[i] = 0;
                }
            }
            // Return demand outputs
            return demandsOut;
        }

        /// Calculate DeltaV vector and update resource demand from mass (demandMass)
        public virtual Vector3d CalculateDeltaVV(double m0, double dT, float thrust, float isp, Vector3d thrustUV, out double demandMass)
        {
            // Mass flow rate
            var mdot = thrust / (isp * 9.81f);
            // Change in mass over time interval dT
            var dm = mdot * dT;
            // Resource demand from propellants with mass
            demandMass = dm / densityAverage;
            // Mass at end of time interval dT
            var m1 = m0 - dm;
            // deltaV amount
            var deltaV = isp * 9.81f * Math.Log(m0 / m1);
            // Return deltaV vector
            return deltaV * thrustUV;
        }

        /// Make "engine" and "engineFX" fields refer to the ModuleEngines and ModuleEnginesFX modules in part.Modules
        private void FindModuleEngines()
        {
            foreach (PartModule pm in part.Modules)
            {
                if (pm is MultiModeEngine)
                {
                    multiMode = pm as MultiModeEngine;
                    IsMultiMode = true;
                }
                else if (pm is ModuleEngines)
                {
                    engine = pm as ModuleEngines;
                    IsPersistentEngine = true;
                }
            }

            if (!IsPersistentEngine)
            {
                Debug.Log("[PersistentThrust] No ModuleEngine found.");
            }
        }

        /// Finds the active engine module from the MultiModeEngine partmodule
        private void FetchActiveMode()
        {
            engine = multiMode.runningPrimary ? multiMode.PrimaryEngine as ModuleEngines : multiMode.SecondaryEngine as ModuleEngines;
        }

        /// Update maximum G threshold for timewarp and persistent thrust/Isp/throttle values
        private void UpdatePersistentParameters()
        {
            // skip some ticks
            if (skipCounter++ < 15) return;

            // we are on the 16th tick
            skipCounter = 0;

            // Force KSP to allow time warp during acceleration
            TimeWarp.GThreshold = (engine.maxThrust / this.vessel.GetTotalMass()) / 9.81f + 1f;

            // Update values to use during timewarp
            // Get Isp
            IspPersistent = engine.realIsp;
            // Get throttle
            ThrottlePersistent = vessel.ctrlState.mainThrottle;
            // Get final thrust
            ThrustPersistent = engine.finalThrust;
        }
    }
}
