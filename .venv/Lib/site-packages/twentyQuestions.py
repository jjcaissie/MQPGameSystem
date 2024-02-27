import random

global askedQuestions, fruitToGuess, numQuestions, fruits, gameState, userQuestions, gameData
numQuestions = 5
numGuesses = 0
gameState = 1
gameData = ('','')

#Would be good to convert this into a matrix at some point with either 1 or 0 for any given trait
fruits = {
    'apple':      {'red outside', 'white inside', 'round', 'crisp', 'crunchy', 'temperate', 'waxy', 'smooth', 'skin', 'medium', 'tree'},
    'rasberry':   {'red outside', 'red inside', 'cluster', 'juicy', 'tender', 'subtropical', 'temperate', 'small', 'bush'},
    'pear':       {'green outside', 'white inside', 'oblong', 'crisp', 'crunchy', 'temperate', 'skin', 'medium', 'tree'},
    'watermelon': {'green outside', 'red inside', 'round', 'juicy', 'subtropical', 'thick', 'waxy', 'rind', 'large', 'vine'},
    'honeydew':   {'green outside', 'white inside', 'green inside', 'round', 'juicy', 'subtropical', 'thick', 'waxy', 'rind', 'large', 'vine'},
    'orange':     {'orange outside', 'orange inside', 'round', 'juicy', 'pulpy', 'tender', 'subtropical', 'thick', 'porous', 'peel', 'medium', 'tree'},
    'kumquat':    {'orange outside', 'orange inside', 'round', 'juicy', 'pulpy', 'tender', 'tart', 'subtropical', 'porous', 'small', 'peel', 'tree'},
    'grape':      {'purple outside', 'clear inside', 'round', 'juicy', 'tart', 'tender', 'temperate', 'waxy', 'smooth', 'skin', 'small', 'vine'},
    'plum':       {'purple outside', 'clear inside', 'red inside', 'yellow inside', 'tender', 'round', 'juicy', 'tart', 'temperate', 'waxy', 'smooth', 'skin', 'medium', 'tree'},
    'pineapple':  {'yellow outside', 'yellow inside', 'long', 'juicy', 'tart', 'tropical', 'rind', 'large', 'bush', 'spiky'},
    'banana':     {'yellow outside', 'yellow inside', 'long', 'creamy', 'tropical', 'thick', 'peel', 'medium', 'tree'} #Yes I know banana trees aren't actually trees but the average person does not know that
}

userQuestions = {}

#Decides action based on gameState
def GameManager(userQuestion):
    global gameState
    if gameState == 1:
        return AskQuestion(userQuestion)
    if gameState == 2:
        if(userQuestion == 'y'):    #user selected to continue
            return 'TwentyQuestions'
        elif(userQuestion == 'n'):  #user selected to discontinue
            return 'index'
        else:                       #user is submitting a guess
            return GuessFruit(userQuestion)

#Starts game of 20 Questions with the human as the guesser
def StartGame(charNotToPick = ''):
    global fruitToGuess, fruitAttributes, askedQuestions
    global numGuesses, gameState, userQuestions, gameData
    fruitToGuess, fruitAttributes = GetFruit(charNotToPick)
    gameData = ('','')
    askedQuestions = -1
    numGuesses = 0
    gameState = 1
    userQuestions = {}
    return "I'm thinking of a fruit. Guess which one!"

#Handles user's submitted questions
def AskQuestion(userQuestion):
    global askedQuestions, numQuestions, gameState, userQuestions, gameData
    askedQuestions += 1    

    if(askedQuestions >= numQuestions):                     #check if game should end based on number of questions
        gameState = 2
        answer = "You're out of guesses! Who do you think I am thinking of?"
        gameData = (userQuestion,answer)                    #Store in tuple for personality bot
        return (answer, True, False, False)
    else:                                                   #user has questions left
        #determine if attribute is true for fruit
        answer = "No"
        if(userQuestion in fruitAttributes):
            answer = "Yes"
        gameData = (userQuestion,answer)                    #Store in tuple for personality bot
        answer = userQuestion + " : " + answer
        userQuestions.update({userQuestion: answer})        #add user question and CPU answer to dict to show user at the end
        return (answer, False, False, False)

#Handles when the user guesses the fruit
def GuessFruit(userGuess):
    global numGuesses, userQuestions, fruits
    answer = "You guessed it! I was thinking of " + fruitToGuess + "! Would you like to continue playing?"
    if(userGuess.lower() != fruitToGuess):
        if(numGuesses == 0):
            answer = "That's not it! I think you need my help."
            numGuesses+=1
            return (answer, False, True, fruits, userQuestions)
        answer = "That's not it! I was thinking of " + fruitToGuess +"! Would you like to continue playing?"
        return (answer, False, False, True)     #User did not guess correctly on second try
    return (answer, False, False, True)         #User guessed correctly

#Gets fruit except for the one passed in. Useful so CPU does not pick same fruit twice
def GetFruit(notToPick):
    global fruits
    gotNewFruit = False
    while(not gotNewFruit):
        char = random.choice(list(fruits.keys()))
        if(char != notToPick):
            return char, fruits[char]

#Skips rest of user's questions (User is ready to guess fruit)
def SkipQuestions():
    global askedQuestions, numQuestions
    askedQuestions = numQuestions
