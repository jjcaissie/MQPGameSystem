import tracery
from tracery.modifiers import base_english
import json
from traceryHandles import traceryToDict
from traceryHandles import jsonToDict

responses = jsonToDict("personalityResponses.json")

responsesGrammar = tracery.Grammar(responses)
responsesGrammar.add_modifiers(base_english)

def generateResponse():
    #currently just testing to see what it will even do
    text = responsesGrammar.flatten("#random-response#")
    return text