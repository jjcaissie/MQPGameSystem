import os
import sys
from discord.ext import commands
import discord
from dotenv import load_dotenv
import tracery
from tracery.modifiers import base_english
import json

def traceryToDict(fileName):

    try:
        f = open(fileName, 'r')
        traceryContent = f.read()
        f.close()

        formatted = json.loads(traceryContent)
        return formatted

    except:
        print("traceryToDict() failed")


def jsonToDict(fileName):
    currentDirectory = os.path.dirname(os.path.abspath(sys.argv[0]))
    try:
    	f = open(currentDirectory+"\\"+fileName, 'r')
    except:
        f = open(currentDirectory+"/"+fileName, 'r')
    traceryContent = f.read()
    f.close()

    formatted = json.loads(traceryContent)

    return formatted




