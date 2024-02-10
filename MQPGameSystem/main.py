import myAPP
import time
import threading

gameState = 0
chatbotState = 1

def main():
    currentState = gameState
    usrQcpuA = myAPP.getUsrQcpuA()
    cpuQusrA = myAPP.getCpuQusrA()
    gameThread = threading.Thread(target=myAPP.HostServer)
    gameThread.start()
    while True:
        if currentState == gameState:
            myAPP.isServerPaused = False
            if myAPP.gameState == myAPP.usrGuessingState:
                #print (usrQcpuA[0] + ":" + usrQcpuA[1] + "  :  " + myAPP.getUsrQcpuA()[0] + ":" + myAPP.getUsrQcpuA()[1])
                if usrQcpuA != myAPP.getUsrQcpuA():
                    myAPP.isServerPaused = True
                    usrQcpuA = myAPP.getUsrQcpuA()
                    print(usrQcpuA)
                    currentState = chatbotState
            elif myAPP.gameState == myAPP.cpuGuessingState:
                #print (cpuQusrA[0] + ":" + cpuQusrA[1] + "  ,  " + myAPP.getCpuQusrA()[0] + ":" + myAPP.getCpuQusrA()[1])
                if cpuQusrA != myAPP.getCpuQusrA():
                    myAPP.isServerPaused = True
                    cpuQusrA = myAPP.getCpuQusrA()
                    print(cpuQusrA)
                    currentState = chatbotState
            else: 
                print("GAMESTATE NOT FOUND")
        elif currentState == chatbotState:
            #Simulates personalityState
            print("chatbot state")
            time.sleep(10)
            print("chatbot state end")
            currentState = gameState
        else:
            print("STATE NOT FOUND")
        
main()