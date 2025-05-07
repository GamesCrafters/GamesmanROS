#!/usr/bin/env python3

import requests
from centers import get_centers, get_pickup, get_capture, get_piece_ARTag_frame
from robotControl import getType
# from findPiece import PieceFinder

URL = "https://nyc.cs.berkeley.edu/universal/v1/"

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
starting_position = variants_data["startPosition"]
##############################################################################

############## Get Human or Robot #############################
print("Human or Robot (Enter 'h' or 'r')")
humanA = input("Player 1: ") == "h"
humanB = input("Player 2: ") == "h"
##############################################################################


############################# Meta Data  #####################################
Static_URL = URL + "/positions/?p="
centers = get_centers()[games_data[user_game]["id"]]
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
    moves_available = []
    print("Moves Available:")
    for i in range(len(moves)):
        moves_available.append(int(moves[i]['autoguiMove'].split('_')[2]))
        print(moves[i]['autoguiMove'].split('_')[2])
    user_input = int(input("Choose an available move: "))
    while user_input not in moves_available:
        user_input = int(input("Choose an available move: "))
    
    return moves_available.index(user_input)
    

################################################################################


# Work here!
Dynamic_URL = Static_URL + starting_position

# List of available moves from starting position
moves_data = requests.get(url=Dynamic_URL).json()['moves']

game = games_data[user_game]["id"]
gameType = getType(game)


robotControl = gameType(game, centers, get_pickup(), get_capture(game))

A_turn = True
while (len(moves_data) > 0):
    if A_turn:
        if humanA:
            index = process_human_player_keyboard(moves_data)
            move = moves_data[index]['autoguiMove']
            new_position = moves_data[index]['position']
        else:
            move = pick_best_move(moves_data)
            new_position = pick_best_position(moves_data)
            move_coords = robotControl.processMove(move, [starting_position, new_position])

        print("A : ", move_coords)

        Dynamic_URL = Static_URL + new_position
        moves_data = requests.get(url=Dynamic_URL).json()['moves']
        starting_position = new_position
        A_turn = False
    else:
        if humanB:
            index = process_human_player_keyboard(moves_data)
            move = moves_data[index]['autoguiMove']
            new_position = moves_data[index]['position']
        else:
            move = pick_best_move(moves_data)
            new_position = pick_best_position(moves_data)
            move_coords = robotControl.processMove(move, [starting_position, new_position])

        print("B : ", move_coords)
        
        Dynamic_URL = Static_URL + new_position
        moves_data = requests.get(url=Dynamic_URL).json()['moves']
        starting_position = new_position
        A_turn = True
