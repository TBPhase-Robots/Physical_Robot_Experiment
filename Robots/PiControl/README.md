# pi_control

Contains the code for the Polulu 3pi.
There is code for the wheel encoders and then kinematics calculations from this encoder information.
The webcam sends a pose message to the M5Core2 which will write to the Polulu via serial what angle is it facing. This angle will overwrite the kinematics angle.
The M5Core will also give the Polulu a vector to head towards. Between angle updates the M5Core will use its encoders to head towards the vector direction using a PID controller.
Once it is facing the vector it will go forward until it is no longer facing the correct location via either drift or change of command

# To use

Open pi_control.ino in arduino. If the repository is correctly downloaded two files named "encoders.h" and "kinematics.h" should appear alongside it.
To upload: Go to tools -> go to board -> set to Arduino Leonardo and upload.
