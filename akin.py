import akinator

aki = akinator.Akinator()
difficulty = 80     #MAX IS 80
global askedQuestions, gameState, question, numQuestions, gameData
numQuestions = 20
gameData = ('','')
gameState = 0
#Here as well as in akiStartGame() in order to load page quickly
question = aki.start_game('en', True)   
askedQuestions = 0

def getNumQuestions():
    return numQuestions
def getAskedQuestions():
    return askedQuestions

#Decides action based on gameState
def GameManager(userAnswer):
    global gameState    
    if gameState == 1:                          #game in progress
        return ProgressGame(userAnswer)
    elif gameState == 2:                        #checking user guess
        return IsRight(userAnswer)
    elif gameState == 3:                        #end message and replay
        return ContinueGame(userAnswer)
    else:
        return StartGame()

#Starts game of 20 Questions with the robot as the guesser
def StartGame():
    global gameState, picture, askedQuestions
    global question, aki, difficulty, gameData
    gameData = ("","")
    gameState = 1
    picture = None

    try:
        if(askedQuestions > 0):
            askedQuestions = 0
            aki = akinator.Akinator()
            question = aki.start_game('en', True)
    except:                 #Handle aki timeout error
        aki = akinator.Akinator()
        question = aki.start_game('en', True)
    return question
    
#Gets next question as well as gives user answer to aki. 
#If user out of questions, change to end game screen
#Input: String: users answer in the form of 'y' for yes, 'n' for no, 'idk' for i don't know, 'probably', 'probably not'
#Output: Page that displays akinator's returned output
def ProgressGame(answer):
    global numQuestions, askedQuestions, aki, difficulty, gameData, question
    askedQuestions += 1
    if aki.progression == None:
        return StartGame()                                                      #Ends old game and starts new one
    elif aki.progression <= difficulty and numQuestions > askedQuestions:         #check if game should end based on number of questions
        gameData = (question,answer)
        question = aki.answer(answer)
        return (question, True, True)
    else:
        question = aki.answer(answer)
        return __EndGame()

#Handle akinator's guesses
def __EndGame():
    global gameState
    gameState = 2
    aki.win()

    #filter innapropriate answers out
    valid_guess = -1
    for count, guess in enumerate(aki.guesses):
        if 'porn' not in guess['name'].lower() and 'porn' not in guess['description'].lower() and 'akinator' not in guess['name'].lower() and 'akinator' not in guess['description'].lower():
           valid_guess = count
           break
        else:
            print(guess['name'], guess ['description'], guess['absolute_picture_path'])
    
    #Handle case where only innapropriate guesses are given
    if(valid_guess == -1):
        return ("Were you thinking of something innapropriate?", True, False)
    #Format answer and description
    picture = aki.guesses[valid_guess]['absolute_picture_path']
    question = "Were you thinking of: " + aki.guesses[valid_guess]['name'] + " " + aki.guesses[valid_guess]['description'] + "? "
    return (question, False, False, picture)

#check if robot guessed what the user was thinking
#Input: String: 'y' for yes or any other string for no
#Output: String: "Yay!" if user inputs 'y' or "Dang!" otherwise
def IsRight(question):
    global gameState
    gameState = 3
    if question == "y" or question =="probably":
        question = "Yay!"
    else:
        question = "Dang!"
    question += "\nWould you like to continue playing?"
    return (question, True, False, True)


#Prompt user to replay
#Input: String: 'y' for yes or any other string to return to homepage
#Output: redirect to TwentyQuestionsTwo or homepage
def ContinueGame(question):
    if question == 'y':
        return 'TwentyQuestionsTwo'
    else:
        return 'index'