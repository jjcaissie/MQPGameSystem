import gameSystem
import platform
import threading
import personalityRunner

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

__stateGame = 0
__stateChatbot = 1

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
    global gameData, chatbotData, confidenceValue, changeInConfidenceValue
    currentState = __stateChatbot
    gameThread = threading.Thread(target=gameSystem.HostServer)
    gameThread.start()

    while True:
        while currentState == __stateGame:                              #If in one of the game states
            gameSystem.setChatbotResponse(chatbotData)                  #Send chatbot response to gameSystem
            sendEvent.set()                                             #Signal that data has been sent
            if gameData != gameSystem.getData():                                #Flag for when data is altered WHAT IF DATA IS THE SAME?               
                getEvent.wait()                                                 #Data is ready to be retrieved
                getEvent.clear()
                gameData = gameSystem.getData()                                 #Read data from game
                GetConfidenceValue()                                            #Get confidence value
                print("CHANGE CONFIDENCE VALUE")
                print(changeInConfidenceValue)
                print("CURRENT CONFIDENCE VALUE")
                print(confidenceValue)

                print(chatbotData)
                currentState = __stateChatbot                                   #Give control to chatbot

        while currentState == __stateChatbot:                           #If in the chatbot state
            if gameSystem.gameState == gameSystem.usrGuessingState:         #If game state is in cpu guess state
                chatbotData = personalityRunner.generateResponse(confidenceValue, changeInConfidenceValue, numQuestions(), askedQuestions())
                if __name__ == '__main__':
                    try:
                        if(platform.system() == "Linux"):
                            __publishToNode__(chatbotData)
                    except rospy.ROSInterruptException:
                        pass

            elif gameSystem.gameState == gameSystem.cpuGuessingState:   #If game state is in usr guess state
                chatbotData = personalityRunner.generateResponse(confidenceValue, changeInConfidenceValue, numQuestions(), askedQuestions())
                if __name__ == '__main__':
                    try:
                        if(platform.system() == "Linux"):
                            __publishToNode__(chatbotData)
                    except rospy.ROSInterruptException:
                        pass
            currentState = __stateGame                                  #Give control to game system

main()