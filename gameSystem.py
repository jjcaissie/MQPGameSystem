#This document contains the boiler plate stuff for the flask server as well as handling messages (user input) from the website

from flask import Flask, render_template, request, redirect, url_for
import threading
import akin
import twentyQuestions

app = Flask(__name__)
app.secret_key ='1082rb1p298v1nm28934yvnp1812vg790n01vnh8912vr7nh'

sendEvent = threading.Event()
getEvent = threading.Event()

cpuGuessingState = 0
usrGuessingState = 1
chatbotResponse = ''
gameState = cpuGuessingState
isServerPaused = False

def setChatbotResponse(string):
    global chatbotResponse
    chatbotResponse = string

def getData():
    if gameState == cpuGuessingState:
        return akin.gameData
    else:
        return twentyQuestions.gameData
    
def GetConfidenceValue():
    return akin.aki.progression

@app.route("/")
def index():
    return render_template("index.html")

#Loads games page as well as handles user messages on the games page.
@app.route("/games.html", methods=["POST", "GET"])
def games():
    if request.method == "POST":
        #Gets desired number of questions to play TwentyQuestions with
        numQuestions = int(request.form["difficulty"])
        akin.numQuestions = numQuestions
        twentyQuestions.numQuestions = numQuestions
        return redirect(url_for('TwentyQuestions'))
    return render_template("games.html")

#Loads help page
@app.route("/help.html")
def help():
    return render_template("help.html")

#Loads settings page
@app.route("/settings.html")
def settings():
    return render_template("settings.html")

#Loads TwentyQuestions page where CPU is guessing
@app.route("/TwentyQuestions.html", methods=["POST", "GET"])
def TwentyQuestions():
    global gameState, isServerPaused
    gameState = cpuGuessingState
    if request.method == "POST":
        user_answer = request.form["userAnswer"]
        data = akin.GameManager(user_answer)
        sendEvent.set()             #Signal that data is ready to send
        getEvent.wait()             #Signal that data has been retrieved
        getEvent.clear()            #Reset signal
        #render or redirect HTML 
        if isinstance(data, str):
            del akin.aki
            return redirect(url_for(data))
        elif(len(data) == 3):       #If user asked question
            return render_template("TwentyQuestions.html", question=chatbotResponse+'<br>'+data[0], hidePicture = data[1], showAllButtons = data[2])
        else:                       #If akin is guessing
            return render_template("TwentyQuestions.html", question=chatbotResponse+'<br>'+data[0], hidePicture = data[1], showAllButtons = data[2], picture = data[3])
    else:
        game_thread = threading.Thread(target=akin.StartGame)
        game_thread.start()
        question = akin.StartGame()
        return render_template("TwentyQuestions.html", question=chatbotResponse+'<br>'+question, hidePicture = True, showAllButtons = True)

#Loads TwentyQuestions page where user is guessing
@app.route("/TwentyQuestionsTwo.html", methods=["POST", "GET"])
def TwentyQuestionsTwo():
    global gameState, isServerPaused
    gameState = usrGuessingState
    if request.method == "POST":
        userQuestion = request.form["question"]
        userSkipped = request.form["skip"]
        if(userSkipped == "True"):
            twentyQuestions.SkipQuestions()
        data = twentyQuestions.GameManager(userQuestion)
        sendEvent.set()
        getEvent.wait()
        if isinstance(data, str):
            return redirect(url_for(data))
        elif(len(data) == 4):
            return render_template("TwentyQuestionsTwo.html", answer=chatbotResponse+'<br>'+data[0], firstGuess=data[1], secondGuess=data[2], promptReplay = data[3])
        else:
            return render_template("TwentyQuestionsTwo.html", answer=chatbotResponse+'<br>'+data[0], firstGuess=data[1], secondGuess=data[2], fruits=data[3],userQuestions=data[4])
    data = twentyQuestions.StartGame()
    return render_template('TwentyQuestionsTwo.html', answer=chatbotResponse+'<br>'+data)

#Hosts on local network on port 80
def HostServer():
    app.run(host="0.0.0.0", port=80)
    ShutdownServer()

def ShutdownServer():
    print("Shutting down")
    #Put code here at some point


