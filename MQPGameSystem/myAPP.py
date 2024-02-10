#This document contains the boiler plate stuff for the flask server as well as handling messages (user input) from the website

from flask import Flask, render_template, request, redirect, url_for
import threading
import akin
import twentyQuestions

app = Flask(__name__)
app.secret_key ='1082rb1p298v1nm28934yvnp1812vg790n01vnh8912vr7nh'

@app.route("/")
def index():
    return render_template("index.html")

#Loads games page as well as handles user messages on the games page.
@app.route("/games.html", methods=["POST", "GET"])
def games():
    if request.method == "POST":
        #Gets desired number of questions to play TwentyQuestions with
        global numQuestions, akin
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
    global akin
    if request.method == "POST":
        user_answer = request.form["userAnswer"]
        return akin.GameManager(user_answer)
    else:
        game_thread = threading.Thread(target=akin.StartGame)
        game_thread.start()
        return akin.StartGame()

#Loads TwentyQuestions page where user is guessing
@app.route("/TwentyQuestionsTwo.html", methods=["POST", "GET"])
def TwentyQuestionsTwo():
    if request.method == "POST":
        userQuestion = request.form["question"]
        userSkipped = request.form["skip"]
        print(userQuestion)
        if(userSkipped == "True"):
            twentyQuestions.SkipQuestions()
        return twentyQuestions.GameManager(userQuestion)
    return twentyQuestions.StartGame()

#Hosts on local network on port 80
app.run(host="0.0.0.0", port=80)