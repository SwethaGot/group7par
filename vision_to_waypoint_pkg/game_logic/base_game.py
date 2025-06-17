import tkinter as tk
import random

# --- Game Data Classes ---

class Block:
    def __init__(self, card_label, qr_code):
        self.card_label = card_label
        self.qr_code = qr_code
        self.used = False

class Player:
    def __init__(self, name, symbol, color):
        self.name = name
        self.hand = []
        self.symbol = symbol
        self.color = color

class GameBoard:
    def __init__(self):
        self.grid = [['' for _ in range(5)] for _ in range(4)]
        self.label_map = [['' for _ in range(5)] for _ in range(4)]
        self.wild_positions = {(0,0), (0,4), (3,0), (3,4)}

# --- Game Logic ---

def check_win(board, symbol):
    grid = board.grid
    for i in range(4):
        for j in range(3):
            if grid[i][j] == grid[i][j+1] == grid[i][j+2] == symbol:
                return True
    for i in range(2):
        for j in range(5):
            if grid[i][j] == grid[i+1][j] == grid[i+2][j] == symbol:
                return True
    for i in range(2):
        for j in range(3):
            if grid[i][j] == grid[i+1][j+1] == grid[i+2][j+2] == symbol:
                return True
            if grid[i+2][j] == grid[i+1][j+1] == grid[i][j+2] == symbol:
                return True
    return False

# --- GUI Logic ---

class SequenceGameTrueLogic:
    def __init__(self, root):
        self.root = root
        self.root.title("Sequence Game: Human vs Robot")
        self.board = GameBoard()
        self.create_blocks()
        self.robot = Player("Robot", "R", "lightblue")
        self.human = Player("Human", "H", "lightgreen")
        self.selected_card = None
        self.turn = 1  # Start with human
        self.create_widgets()
        self.assign_board_labels()
        self.start_game()

    def create_blocks(self):
        labels = [f"{chr(65 + i)}{j+1}" for i in range(4) for j in range(4)]  # A1 to D4
        random.shuffle(labels)
        self.blocks = [Block(label, f"QR_{label}") for label in labels]

    def assign_board_labels(self):
        # Assign the 16 alphanumeric labels to non-corner tiles
        available_positions = [(i, j) for i in range(4) for j in range(5) if (i, j) not in self.board.wild_positions]
        labels = [f"{chr(65 + i)}{j+1}" for i in range(4) for j in range(4)]  # A1 to D4
        random.shuffle(labels)
        for pos, label in zip(available_positions, labels):
            i, j = pos
            self.board.label_map[i][j] = label
            self.buttons[i][j].config(text=label)

        # Mark wild positions
        for i, j in self.board.wild_positions:
            self.buttons[i][j].config(text="SEQ", bg="gray")

    def create_widgets(self):
        self.buttons = []
        for i in range(4):
            row = []
            for j in range(5):
                btn = tk.Button(self.root, text="", width=8, height=4,
                                font=('Arial', 12, 'bold'),
                                command=lambda i=i, j=j: self.handle_click(i, j))
                btn.grid(row=i, column=j)
                row.append(btn)
            self.buttons.append(row)

        self.status_label = tk.Label(self.root, text="", font=('Arial', 14))
        self.status_label.grid(row=5, column=0, columnspan=5)

        self.card_buttons_frame = tk.Frame(self.root)
        self.card_buttons_frame.grid(row=6, column=0, columnspan=5)

    def start_game(self):
        for _ in range(2):
            self.robot.hand.append(self.blocks.pop())
            self.human.hand.append(self.blocks.pop())
        self.update_hand_buttons()
        self.status_label.config(text="Your turn! Select a card and click its matching tile.")

    def update_hand_buttons(self):
        for widget in self.card_buttons_frame.winfo_children():
            widget.destroy()
        for idx, card in enumerate(self.human.hand):
            btn = tk.Button(self.card_buttons_frame, text=card.card_label,
                            font=('Arial', 12), width=10,
                            command=lambda i=idx: self.select_card(i))
            btn.pack(side='left', padx=10)

    def select_card(self, index):
        self.selected_card = self.human.hand.pop(index)
        self.status_label.config(text=f"Selected: {self.selected_card.card_label}. Now click its matching tile.")

    def handle_click(self, i, j):
        if self.turn % 2 == 1 and self.selected_card:
            if self.board.grid[i][j] != '':
                self.status_label.config(text="Tile already occupied.")
                return
            if self.board.label_map[i][j] != self.selected_card.card_label and (i, j) not in self.board.wild_positions:
                self.status_label.config(text="Incorrect tile for selected card.")
                return
            self.place_piece(i, j, self.human, self.selected_card.card_label)
            self.selected_card = None
            if check_win(self.board, self.human.symbol):
                self.status_label.config(text="You Win!")
                return
            if self.blocks:
                self.human.hand.append(self.blocks.pop())
            self.update_hand_buttons()
            self.turn += 1
            self.root.after(1000, self.robot_turn)

    def robot_turn(self):
        placed = False
        chosen = self.robot.hand.pop(0)

        # Step 1: Try placing on an exact matching card label tile
        for i in range(4):
            for j in range(5):
                if self.board.grid[i][j] == '' and self.board.label_map[i][j] == chosen.card_label:
                    self.place_piece(i, j, self.robot, chosen.card_label)
                    placed = True
                    break
            if placed:
                break

        # Step 2: If no exact match, try placing on a SEQ tile
        if not placed:
            for i, j in self.board.wild_positions:
                if self.board.grid[i][j] == '':
                    self.place_piece(i, j, self.robot, chosen.card_label)
                    placed = True
                    break

        # Step 3: Finish turn
        if check_win(self.board, self.robot.symbol):
            self.status_label.config(text="Robot Wins!")
            return
        if self.blocks:
            self.robot.hand.append(self.blocks.pop())
        self.turn += 1
        self.status_label.config(text="Your turn! Select a card and click its matching tile.")


    def place_piece(self, i, j, player, label):
        self.board.grid[i][j] = player.symbol
        self.buttons[i][j].config(text=label, bg=player.color, state='disabled')


if __name__ == "__main__":
    root = tk.Tk()
    app = SequenceGameTrueLogic(root)
    root.mainloop()