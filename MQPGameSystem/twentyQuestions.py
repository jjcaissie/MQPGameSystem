from flask import render_template, redirect, url_for
import random

global askedQuestions, characterToGuess, numQuestions, characters, gameState, userQuestions
numQuestions = 5
numGuesses = 0

characters = {
    'mario':        {'man', 'human', 'hero', 'videoGame', 'nintendo', 'job', 'plumber', 'mainCharacter', 'transform'},
    'taylor swift': {'woman', 'human', 'real', 'job', 'singer', 'american'},
    'mickey mouse': {'man', 'cartoon', 'disney', 'mainCharacter', 'american', 'job', 'detective'},
    'jack black':   {'man', 'human', 'real', 'job', 'actor', 'american'},
    'shrek':        {'man', 'dreamworks', 'movie', 'hero', 'mainCharacter', 'transform'},
}

userQuestions = {}

#Decides action based on gameState
def GameManager(userQuestion):
    if gameState == 1:
        return AskQuestion(userQuestion)
    if gameState == 2:
        if(userQuestion == 'y'):    #user selected to continue
            return redirect(url_for('TwentyQuestions'))
        elif(userQuestion == 'n'):  #user selected to discontinue
            return redirect(url_for('index'))
        else:                       #user is submitting a guess
            return GuessCharacter(userQuestion)

#Starts game of 20 Questions with the human as the guesser
def StartGame(charNotToPick = ''):
    global characterToGuess, characterAttributes, askedQuestions, numGuesses, gameState, userQuestions
    characterToGuess, characterAttributes = GetCharacter(charNotToPick)
    askedQuestions = -1
    numGuesses = 0
    gameState = 1
    userQuestions = {}
    answer = "I'm thinking of a character. Guess who!"
    return render_template('TwentyQuestionsTwo.html', answer=answer)

#Handles user's submitted questions
def AskQuestion(userQuestion):
    global askedQuestions, numQuestions, gameState, userQuestions
    askedQuestions += 1    

    if(askedQuestions >= numQuestions):                  #check if game should end based on number of questions
        gameState = 2
        answer = "You're out of guesses! Who do you think I am thinking of?"
        return render_template("TwentyQuestionsTwo.html", answer=answer, firstGuess=True)
    else:                                                #user has questions left
        #determine if attribute is true for character
        answer = "No"
        if(userQuestion in characterAttributes):
            answer = "Yes"
        answer = userQuestion + " : " + answer
        userQuestions.update({userQuestion: answer})      #add user question and CPU answer to dict to show user at the end
        return render_template("TwentyQuestionsTwo.html", answer=answer)

#Handles when the user guesses the character
def GuessCharacter(userGuess):
    global numGuesses, userQuestions, characters
    answer = "You guessed it! I was thinking of " + characterToGuess + "! Would you like to continue playing?"
    if(userGuess.lower() != characterToGuess):
        if(numGuesses == 0):
            answer = "That's not it! I think you need my help."
            numGuesses+=1
            return render_template("TwentyQuestionsTwo.html", answer=answer, secondGuess=True, characters=characters, userQuestions=userQuestions)
        answer = "That's not it! I was thinking of " + characterToGuess +"! Would you like to continue playing?"
        return render_template("TwentyQuestionsTwo.html", answer=answer, promptReplay=True)     #User did not guess correctly on second try
    return render_template("TwentyQuestionsTwo.html", answer=answer, promptReplay=True)         #User guessed correctly

#Gets character except for the one passed in. Useful so CPU does not pick same character twice
def GetCharacter(notToPick):
    global characters
    gotNewCharacter = False
    while(not gotNewCharacter):
        char = random.choice(list(characters.keys()))
        if(char != notToPick):
            return char, characters[char]

#Skips rest of user's questions (User is ready to guess character)
def SkipQuestions():
    global askedQuestions, numQuestions
    askedQuestions = numQuestions