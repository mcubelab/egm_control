MODULE EGM_test_UDP 
     
    VAR egmident egmID1; 
    VAR egmstate egmSt1; 

    CONST egm_minmax egm_minmax_joint1:=[-0.1,0.1]; 
 
    PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[600,0,0],[0.707,0,-0.707,0]]]; 
    PERS tooldata currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]]; 
    
    VAR jointtarget jointsTarget; 
     
    PROC main() 
    	jointsTarget:=CJointT();

        ConfJ\Off; 
        SingArea\Off; 
        MoveAbsJ jointsTarget, v50, fine, currentTool \Wobj:=currentWobj; 
        
        WaitTime 0.5; 
         
        !EXIT; 
        EGMReset egmID1; 
        EGMGetId egmID1; 
        egmSt1 := EGMGetState(egmID1); 
        TPWrite "EGM state: "\Num := egmSt1; 
         
        IF egmSt1 <= EGM_STATE_CONNECTED THEN 
            ! Set up the EGM data source: UdpUc server using device "EGMtest" and configuration "default" 
            EGMSetupUC ROB_1, egmID1, "default", "EGMSensor" \Joint; 
        ENDIF 

        ! Copied examples from IRC5 Application manual
        EGMActJoint egmID1 \J1:=egm_minmax_joint1 \J2:=egm_minmax_joint1 \J3:=egm_minmax_joint1
                           \J4:=egm_minmax_joint1 \J5:=egm_minmax_joint1 \J6:=egm_minmax_joint1
                           \J7:=egm_minmax_joint1 \LpFilter:=20 \MaxSpeedDeviation:=100;

        EGMRunJoint egmID1, EGM_STOP_HOLD \J1 \J2 \J3 \J4 \J5 \J6 \J7 \CondTime:=1000
                           \RampInTime:=0.05 \PosCorrGain:=1;
 
        IF egmSt1 = EGM_STATE_CONNECTED THEN 
            TPWrite "Reset EGM instance egmID1"; 
            EGMReset egmID1;  
        ENDIF  
         
        TPWrite "EGM STOPPED"; 
        WHILE TRUE DO 
            WaitTime 0.5; 
            TPWrite "Reset EGM instance egmID1"; 
             
        ENDWHILE 
    ENDPROC         
  
ENDMODULE