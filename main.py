import gameSystem
import platform
import threading
import personalityRunner
import time

if platform.system() == "Linux":
    import rospy
    from std_msgs.msg import String
    
    #ROS Variables
    pub = rospy.Publisher('publisher', String, queue_size = 10)
    rospy.init_node('publish_node', anonymous=True)
    rate = rospy.Rate(10)

    def __publishToNode__(publishString):
        rospy.loginfo("Data is being sent")
        pub.publish(publishString)
        rate.sleep()

gameData = (" ","")             #Data being sent to personality bot. formatted like (question, answer)
chatbotData = ""                #Data being sent to game system. Should be the robot's response given gameData
confidenceValue = 0.0
changeInConfidenceValue = 0.0
numQuestions = gameSystem.getNumQuestions
askedQuestions = gameSystem.getAskedQuestions

sendEvent = gameSystem.getEvent
getEvent = gameSystem.sendEvent

def GetConfidenceValue():
    global confidenceValue, changeInConfidenceValue   
    
    pastConfidenceValue = confidenceValue                               #Temp store the past confidence value to calc change
    try:
        confidenceValue = gameSystem.GetConfidenceValue()             #Get current confidence value
        if confidenceValue == None:                                         #If type is None,
            confidenceValue = 0.0                                               #then equal to 0
        changeInConfidenceValue = confidenceValue - pastConfidenceValue     #Calculate change in confidence values
    except:        
        changeInConfidenceValue = 0.0
        confidenceValue = 0.0

def main():
    global gameData, chatbotData, confidenceValue
    global changeInConfidenceValue
    sendEvent.clear()
    getEvent.set()
    gameThread = threading.Thread(target=gameSystem.HostServer)
    gameThread.start()
    while True:
        if gameData != gameSystem.getData():                                               
            getEvent.wait()                                                 
            getEvent.clear()
            gameData = gameSystem.getData()                                 
            GetConfidenceValue()                                            
            print("CHANGE CONFIDENCE VALUE: ", changeInConfidenceValue)
            print("CURRENT CONFIDENCE VALUE: ", confidenceValue)
            if(gameData == "makingGuess"):  #Eventually generate different responses
                chatbotData = personalityRunner.generateResponse(confidenceValue, changeInConfidenceValue, numQuestions(), askedQuestions())
            elif(gameData == "playAgain"):
                chatbotData = personalityRunner.continueGame()
            elif(gameSystem.gameState == gameSystem.cpuGuessingState):
                chatbotData = personalityRunner.generateResponse(confidenceValue, changeInConfidenceValue, numQuestions(), askedQuestions())
            else:
                chatbotData = personalityRunner.generateResponse2()
            print(chatbotData)
            gameSystem.setChatbotResponse(chatbotData)                      #Send chatbot response to gameSystem
            try:
                if(platform.system() == "Linux"):
                    __publishToNode__(chatbotData)
            except rospy.ROSInterruptException:
                pass
            sendEvent.set()
main()