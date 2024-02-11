#This document contains the boiler plate stuff for the flask server as well as handling messages (user input) from the website

from flask import Flask, render_template, request, redirect, url_for
import threading
import akin
import twentyQuestions
import time

app = Flask(__name__)
app.secret_key ='1082rb1p298v1nm28934yvnp1812vg790n01vnh8912vr7nh'

cpuGuessingState = 0
usrGuessingState = 1
gameState = cpuGuessingState
isServerPaused = False

def getCpuQusrA():
    return akin.cpuQusrA
def getUsrQcpuA():
    return twentyQuestions.usrQcpuA

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
        time.sleep(1)   #NEED TO CODE ACTUAL SOLUTION AT SOME POINT
        while(isServerPaused == True):
            time.sleep(1)
        if isinstance(data, str):
            return redirect(url_for(data))
        elif(len(data) == 3):
            return render_template("TwentyQuestions.html", question=data[0], hidePicture = data[1], showAllButtons = data[2])
        else:
            return render_template("TwentyQuestions.html", question=data[0], hidePicture = data[1], showAllButtons = data[2], picture = data[3])
    else:
        game_thread = threading.Thread(target=akin.StartGame)
        game_thread.start()
        question = akin.StartGame()
        return render_template("TwentyQuestions.html", question=question, hidePicture = True, showAllButtons = True)

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
        time.sleep(2)   #NEED TO CODE ACTUAL SOLUTION AT SOME POINT
        while(isServerPaused == True):
            time.sleep(1) 
        if isinstance(data, str):
            return redirect(url_for(data))
        elif(len(data) == 4):
            return render_template("TwentyQuestionsTwo.html", answer=data[0], firstGuess=data[1], secondGuess=data[2], promptReplay = data[3])
        else:
            return render_template("TwentyQuestionsTwo.html", answer=data[0], firstGuess=data[1], secondGuess=data[2], characters=data[3],userQuestions=data[4])
    data = twentyQuestions.StartGame()
    return render_template('TwentyQuestionsTwo.html', answer=data)



#Hosts on local network on port 5000
def HostServer():
    app.run(host="0.0.0.0", port=5000)

