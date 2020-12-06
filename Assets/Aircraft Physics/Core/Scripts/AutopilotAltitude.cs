using System;
using UnityEngine;
using UnityEngine.UI;

namespace Aircraft_Physics.Core.Scripts
{
    public class AutopilotAltitude : MonoBehaviour
    {
        //PID parameters
        [SerializeField]private float PGain;
        [SerializeField]private float IGain;
        [SerializeField]private float DGain;
        [SerializeField]private float MinOutput;
        [SerializeField]private float MaxOutput;
        [Range(-1000, 1000)]
        [SerializeField] private float setPoint;
        [SerializeField] private float output;

        //PID object
        private PIDController PID;

        //Other references
        private AirplaneController controller;
        private Rigidbody aircraft;
        [SerializeField] private Slider slider;
        
        private void Awake()
        {
            //setup PID
            PID = new PIDController();
            PID.SetConstants(PGain, IGain, DGain, MinOutput, MaxOutput);
            
            //get other references
            controller = GetComponent<AirplaneController>();
            aircraft = GetComponent<Rigidbody>();
            
            //disable
            enabled = false;
        }

        private void OnEnable()
        {
            PID.Reset();
        }

        private void Update()
        {
            setPoint = slider.value * 100f;
        }

        private void FixedUpdate()
        {
            //update parameters (for tuning)
            PID.SetConstants(PGain, IGain, DGain, MinOutput, MaxOutput);
            
            //run PID controller
            output = -PID_Update(setPoint, GetVerticalSpeed(), Time.fixedDeltaTime) / 10f;
            controller.Pitch = output;
        }

        private float PID_Update(float SetPoint, float ProcessVariable, float DeltaTime)
        {
            //set the goal to whatever the slider is
            PID.SetPoint = SetPoint;

            //set the process variable
            PID.ProcessVariable = ProcessVariable;

            //call the function to run PID
            return PID.ControlVariable(DeltaTime);
        }

        public float GetVerticalSpeed()
        {
            float temp = aircraft.velocity.y * 196.85f; //m/s converted to ft/min (standard aviation unit in the US)
            return temp;
        }
    }
}