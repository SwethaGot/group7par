# This will be the game_logic.py file that contains only game logic, separate from the GUI.
# It defines the data classes, board structure, and robot strategy (win/block/center placement).

class Block:
    def __init__(self, card_label):
        self.card_label = card_label

class Player:
    def __init__(self, name, symbol, color):
        self.name = name
        self.symbol = symbol
        self.color = color
        self.hand = []

class GameBoard:
    def __init__(self):
        self.rows = 5
        self.cols = 4
        self.grid = [['' for _ in range(self.cols)] for _ in range(self.rows)]
        self.label_map = [['' for _ in range(self.cols)] for _ in range(self.rows)]
        self.wild_positions = {(0,0), (0,3), (4,0), (4,3)}

    def reset(self):
        self.grid = [['' for _ in range(self.cols)] for _ in range(self.rows)]
        self.label_map = [['' for _ in range(self.cols)] for _ in range(self.rows)]


def check_win(board, symbol):
    g = board.grid
    for i in range(board.rows):
        for j in range(board.cols - 2):
            if g[i][j] == symbol and g[i][j+1] == symbol and g[i][j+2] == symbol:
                return True
    for i in range(board.rows - 2):
        for j in range(board.cols):
            if g[i][j] == symbol and g[i+1][j] == symbol and g[i+2][j] == symbol:
                return True
    for i in range(board.rows - 2):
        for j in range(board.cols - 2):
            if g[i][j] == symbol and g[i+1][j+1] == symbol and g[i+2][j+2] == symbol:
                return True
            if g[i+2][j] == symbol and g[i+1][j+1] == symbol and g[i][j+2] == symbol:
                return True
    return False

def find_winning_move(board, card_label, symbol):
    g = board.grid
    for i in range(board.rows):
        for j in range(board.cols):
            if (i, j) in board.wild_positions:
                continue
            if board.label_map[i][j] != card_label or g[i][j] != "":
                continue
            g[i][j] = symbol
            if check_win(board, symbol):
                g[i][j] = ""
                return (i, j)
            g[i][j] = ""
    return None

def best_robot_move(board, card_label, robot_symbol, human_symbol):
    # Step 1: Try to win
    win_move = find_winning_move(board, card_label, robot_symbol)
    if win_move:
        return win_move
    # Step 2: Try to block
    block_move = find_winning_move(board, card_label, human_symbol)
    if block_move:
        return block_move
    # Step 3: Prefer center
    candidates = []
    for i in range(board.rows):
        for j in range(board.cols):
            if (i, j) in board.wild_positions:
                continue
            if board.grid[i][j] == "" and board.label_map[i][j] == card_label:
                candidates.append((i, j))
    if candidates:
        candidates.sort(key=lambda pos: abs(pos[0] - 2) + abs(pos[1] - 1))
        return candidates[0]
    return None
