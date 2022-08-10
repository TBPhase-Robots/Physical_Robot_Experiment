from tkinter import *

from pygame import Color

from std_msgs.msg import Int32
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from model.Publisher import Publisher
from model.IntPublisher import IntPublisher
from std_msgs.msg import String
class Window(Frame):


# State Controller is the separate standalone script that is used to send commands to runSimulation over ROS.
# This means it can be run on any computer in the network (including your own)
# Commands include changeing state machine state as well as adding and removing agents.

# Add Agent should only ever be used when runSimulation.py is running locally, ie cfg['event_driven'] and cfg['event_driven_movements'] is set to false


# AVAILABLE STATE COMMANDS TO SET TO runSimulation.py

# states:

# setup_start

# sheep_setup_loop

# dog_setup_loop

# pig_setup_loop

# experiment

    

    def __init__(self, master=None):
        Frame.__init__(self, master)        
        self.master = master

        # widget can take all window
        self.pack(fill=BOTH, expand=1)

        # create button, link it to clickExitButton()
        setupStartButton = Button(self, text="Setup Start and sheep", command=self.clickSetupStartButton)
        #sheepSetupLoopButton = Button(self, text="Sheep setup loop", command=self.clickSheepSetupLoopButton)
        dogSetupLoopButton = Button(self, text="Dog setup loop", command=self.clickDogSetupLoopButton)
        pigSetupLoopButton = Button(self, text="Pig setup loop", command=self.clickPigSetupLoopButton)
        standbySetupLoopButton = Button(self, text="Standby setup loop", command=self.standbySetupLoopButton)
        experimentButton = Button(self, text="Experiment", command=self.clickExperimentButton)

        dispatchButton = Button(self, text="Dispatch", command=self.clickDispatchButton)
        recallButton = Button(self, text="Recall", command=self.clickRecallButton)

        addAgentButton = Button(self, text="Add Agent", command=self.clickAddAgentButton)
        setAgentsToStandbyButton = Button(self, text="SetAllToStandby", command=self.clickStandbyButton)
        sendNewJsonButton = Button(self, text="Set JSON config", command=self.clickSendNewJsonButton)

        self.T = Text(root, height = 3, width = 45)

        self.T.place(x = 50, y = 500)
        # place button at (0,0)
        setupStartButton.place(x=100, y=0)
     #   sheepSetupLoopButton.place(x=100, y = 50)
        dogSetupLoopButton.place(x=100, y= 50)
        pigSetupLoopButton.place(x=100, y=100)
        standbySetupLoopButton.place(x=100, y = 150)
        experimentButton.place(x=100, y=200)
        

        dispatchButton.place(x=100, y=250)
        recallButton.place(x=200, y=250)
        addAgentButton.place(x=200, y = 300)
        setAgentsToStandbyButton.place(x=100, y = 350)
        sendNewJsonButton.place(x=100, y = 400)
        
        # define command publisher
        commandListenerTopicName = "/controller/command"
        dispatchListenerTopicName = "/controller/dispatch"
        agentListenerTopicName = "/global/robots/added"
        jsonListenerTopicName = "/controller/config"
        self.statePublisher = Publisher(commandListenerTopicName) 
        self.dispatchPublisher = Publisher(dispatchListenerTopicName)
        self.agentPublisher = IntPublisher(agentListenerTopicName)
        self.jsonPublisher = Publisher(jsonListenerTopicName)
        self.i = 0



    def clickSendNewJsonButton(self):
        print("sent new json")
        msg = String()
        msg.data = self.T.get("1.0", "end-1c")
        print(msg.data)
        self.jsonPublisher.pub.publish(msg)

    def clickStandbyButton(self):
        msg = String()
        msg.data = "set_to_standby"
        self.statePublisher.pub.publish(msg)
    def clickSetupStartButton(self):

        msg = String()
        msg.data = "setup_start"

        
        self.statePublisher.pub.publish(msg)


        print(msg)

    def clickAddAgentButton(self):
        msg = Int32()
        msg.data = self.i
        self.i += 1

        
        self.agentPublisher.pub.publish(msg)


        print(msg)




    def clickDogSetupLoopButton(self):

        msg = String()
        msg.data = "dog_setup_start"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def clickPigSetupLoopButton(self):

        msg = String()
        msg.data = "pig_setup_start"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def standbySetupLoopButton(self):

        msg = String()
        msg.data = "standby_setup_start"
        self.statePublisher.pub.publish(msg)
        print(msg)

    def clickExperimentButton(self):

        msg = String()
        msg.data = "experiment"

        
        self.statePublisher.pub.publish(msg)

        print(msg)

    def clickDispatchButton(self):

        msg = String()
        msg.data = "dispatch"

        
        self.dispatchPublisher.pub.publish(msg)

        print(msg)

    def clickRecallButton(self):

        msg = String()
        msg.data = "recall"

        
        self.dispatchPublisher.pub.publish(msg)

        print(msg)

        


rclpy.init(args=None)

root = Tk()
app = Window(root)
root.wm_title("Tkinter button")
# Set window resolution
root.geometry("320x600")
root.mainloop()

    
