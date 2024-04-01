import tracery
from tracery.modifiers import base_english
import json
from traceryHandles import traceryToDict
from traceryHandles import jsonToDict

responses = jsonToDict("personalityResponses.json")

responsesGrammar = tracery.Grammar(responses)
responsesGrammar.add_modifiers(base_english)

def generateResponseConfidence(val):
    response = ""
    if(val == 0.0):
        response = "greeting"
    elif(val < 15):    
        response = "concern"
    elif(val < 30):
        response = "impatient"
    elif(val < 45):    
        response = "confused-game"
    elif(val < 55):
        response = "neutral-game"
    elif(val < 75):
        response = "happy-game"
    elif(val >= 75):
        response = "smirk-game"
    return responsesGrammar.flatten(response)

def generateResponseChangeConfidence(val):
    response = ""
    if(val == 0.0):
        response = "greeting"
    elif(val < -15):
        response = "confused-game"
    elif(val < -5):
        response = "concern"
    elif(val < 10):
        response = "neutral-game"
    elif(val < 20):
        response = "happy-game"
    elif(val < 40):
        response = "smirk-game" 
    elif(val >= 40):
        response = "surprised"
    return responsesGrammar.flatten(response)

def generateResponse(confidence, changeConfidence, numQuestions, askedQuestions):
    expected_threshold = -80 * (askedQuestions / (2 * numQuestions)) + 80

    print("ASKED QUESTIONS")
    print(askedQuestions)

    print("NUM QUESTIONS")
    print(numQuestions)

    print("EXPECTED THRESHOLD:")
    print(expected_threshold)

    print("CONF VAL")
    print(confidence)

    print("DEL CONF VAL")
    print(changeConfidence)

    text = "no response found"
    if confidence > expected_threshold:
        text = generateResponseConfidence(confidence)
    else:
        text = generateResponseChangeConfidence(changeConfidence)

    return text