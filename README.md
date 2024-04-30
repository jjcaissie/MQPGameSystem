# MQP Game System and Personality Module

All packages should be included. Run main.py to start the script. It will output a link that will bring you to the website

## Main.py
This script acts as the interface between the personality system and the game system
### Variables
**gameData**
Tuple that represends the (question, answer) of the last page of the game
**chatbotData** 
Data that is being sent back to the game data. Currently only holds the personality system's response
**confidenceValue**
The current confidence value of the game system. 0 Represents low confidence while 80+ represents full confidence
**changeInConfidenceValue**
Last confidenceValue - current confidenceValue
**numQuestions**
The number of questions allowed in the running game session
**askedQuestions**
The number of questions that the game system has asked the users
## gameSystem.py
This module holds the game system and flask server. There are 2 main states. The first state is the CPU guessing state where the CPU guesses what the user is guessing, and the second state is the user guessing state where the user guesses what the CPU is thinking
### Variables

**gameState** 
utilizes cpuGuessingState and usrGuessingState to determine which state the game is in (Gets passed to the main.py script)
### Functions
**setChatbotResponse**
Sets the CPU response that will be displayed on the website
**getData**
Returns the data from the current game session
## akin.py
This module is the CPU guessing part of the game system. The CPU attempts to guess what the user is guessing
### Variables

**aki**
An instance of the akinator wrapper
**difficulty**
The confidence value that the system will guess at (Max is 80)
**gameState**
1 = game in progress
2 = game is askign the user if its guess is correct
3 = game has ended. Prompt replay
### Functions
**GameManager**
Uses the user's answer and gameState to decide the next action
**StartGame**
Starts an instance of the akinator game
Resets all variables
**ProgressGame**
Input: String: users answer in the form of 'y' for yes, 'n' for no, 'idk' for i don't know, 'probably', 'probably not'
Passes the user's answer to the akinator instance
Retrieves the next question of the game.
If the CPU has run out of questions, it calls EndGame()
**EndGame**
Gets the akinator win state from the akinator module.
Filters out innapropriate answers
Formats CPU guess and description
**IsRight**
Asks the user if the guess was correct.
Input: String: 'y' for yes or any other string for no
Output: String: "Yay!" if user inputs 'y' or "Dang!" otherwise
**ContinueGame**
Prompt user to replay

## twentyQuestions
### Variables
**fruits**
Table representing the fruits and attributes
**userQuestions**
An array of all the questions the user asked
### Functions
**GameManager**
1 = user asking a question
2 = User is guessing or prompted to return to main menu
**StartGame**
Selects a fruit for the user to guess
Resets all variables
**AskQuestion**
Handles the user's submitted questions
Checks if user has run out of questions. 
Sets gameData variable
returns the CPUs answer to be displayed.
**GuessFruit**
Handles the user's guess when trying to guess the right fruit
**GetFruit**
Gets a random fruit
**SkipQuestion**
Skips to the gues screen
