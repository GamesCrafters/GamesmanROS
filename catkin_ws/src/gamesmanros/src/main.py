#!/usr/bin/env python3

import requests
import rospy
from robotControl import getType

URL = "https://nyc.cs.berkeley.edu/universal/v1/"
vision = False
rospy.init_node("main_game_node", anonymous=True)

# Don't TOUCH Code blocks below!
######################### Get All Games #####################################
games_data = requests.get(url=URL).json()
for i in range(len(games_data)):
    print(i, " : ", games_data[i]['name'])
user_game = int(input("Pick the index of the game you want to play: "))

print("Game Chosen: ", games_data[user_game]["id"])
URL = URL + games_data[user_game]["id"] + '/'
#############################################################################

############## Get Variant and Starting Positon #############################
variants_data = requests.get(url=URL).json()['variants']
for j in range(len(variants_data)):
    print(j, " : ", variants_data[j]["id"])
user_variant = int(input("Pick the index of the variant you want to play: "))
variant = variants_data[user_variant]["id"]
URL = URL + variant + '/'
variants_data = requests.get(url=URL).json()
current_position = variants_data["startPosition"]
##############################################################################

################## Get Human or Robot ########################################
print("Human or Robot (Enter 'h' or 'r')")
humanA = input("Player 1: ") == "h"
humanB = input("Player 2: ") == "h"

if humanA or humanB:
    move_value = input("Move Value [y or n]: ") == 'y'
##############################################################################

###############################################################################
def pick_best_move(moves):
    position_values = {}
    for i in range(len(moves)):
        if moves[i]['moveValue'] not in position_values:
            position_values[moves[i]['moveValue']] = [moves[i]['autoguiMove']]
        else:
            position_values[moves[i]['moveValue']].append(moves[i]['autoguiMove'])

    if 'win' in position_values and len(position_values['win']) > 0:
        return position_values['win'][0]
    elif 'draw' in position_values and len(position_values['draw']) > 0:
        return position_values['draw'][0]
    elif 'lose' in position_values and len(position_values['lose']) > 0:
        return position_values['lose'][0]
    else:
        print('error: in pick_best_move')
        exit()

def pick_best_position(moves):
    position_values = {}
    for i in range(len(moves)):
        if moves[i]['moveValue'] not in position_values:
            position_values[moves[i]['moveValue']] = [moves[i]['position']]
        else:
            position_values[moves[i]['moveValue']].append(moves[i]['position'])

    if 'win' in position_values and len(position_values['win']) > 0:
        return position_values['win'][0]
    elif 'draw' in position_values and len(position_values['draw']) > 0:
        return position_values['draw'][0]
    elif 'lose' in position_values and len(position_values['lose']) > 0:
        return position_values['lose'][0]
    else:
        print('error: in pick_best_postion')
        exit()

def process_human_player_keyboard(moves):
    print("Moves Available:")
    for i in range(len(moves)):
        before, after = (moves[i]['autoguiMove'].split('_')[1], moves[i]['autoguiMove'].split('_')[2])
        print(str(i) + " : ", before, after, moves[i]['moveValue'] if move_value else '')
    index = int(input("Choose an available move: "))
    
    return (moves_data[index]['autoguiMove'], moves_data[index]['position'])
################################################################################

############################# Meta Data  #####################################
Static_URL = URL + "/positions/?p="
##############################################################################

Dynamic_URL = Static_URL + current_position

# List of available moves from starting position
moves_data = requests.get(url=Dynamic_URL).json()['moves']

game = games_data[user_game]["id"]
gameType = getType(game)

robotControl = gameType(game, vision)

A_turn = True
while (len(moves_data) > 0):
    if A_turn:
        if humanA:
            move, new_position = process_human_player_keyboard(moves_data)
        else:
            move = pick_best_move(moves_data)
            new_position = pick_best_position(moves_data)
            move_coords = robotControl.processMove(move, [current_position, new_position])

        print("A : ", move)
        print("A : ", move_coords)

        Dynamic_URL = Static_URL + new_position
        moves_data = requests.get(url=Dynamic_URL).json()['moves']
        current_position = new_position
        A_turn = False
    else:
        if humanB:
            move, new_position = process_human_player_keyboard(moves_data)
        else:
            move = pick_best_move(moves_data)
            new_position = pick_best_position(moves_data)
            move_coords = robotControl.processMove(move, [current_position, new_position])

        print("B : ", move)
        print("B : ", move_coords)
        
        Dynamic_URL = Static_URL + new_position
        moves_data = requests.get(url=Dynamic_URL).json()['moves']
        current_position = new_position
        A_turn = True
