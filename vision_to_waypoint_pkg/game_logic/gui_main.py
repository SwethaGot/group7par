import tkinter as tk
import random
from game_logic import Block, Player, GameBoard, check_win, best_robot_move

class SequenceGame:
    def __init__(self, root):
        self.root = root
        self.root.title("Sequence Game: Human vs Robot")
        self.board = GameBoard()
        self.buttons = []
        self.selected_card = None
        self.turn = 1

        self.robot = Player("Robot", "R", "lightblue")
        self.human = Player("Human", "H", "lightgreen")

        self.fixed_labels = [
            ["SEQ", "A1", "A2", "SEQ"],
            ["B1", "B2", "B3", "B4"],
            ["C1", "C2", "C3", "C4"],
            ["D1", "D2", "D3", "D4"],
            ["SEQ", "E1", "E2", "SEQ"]
        ]

        self.create_widgets()
        self.assign_fixed_labels()
        self.create_deck()
        self.start_game()

    def create_widgets(self):
        for i in range(self.board.rows):
            row = []
            for j in range(self.board.cols):
                btn = tk.Button(self.root, text="", width=8, height=4,
                                font=('Arial', 12, 'bold'),
                                command=lambda i=i, j=j: self.handle_click(i, j))
                btn.grid(row=i, column=j)
                row.append(btn)
            self.buttons.append(row)

        self.status_label = tk.Label(self.root, text="", font=('Arial', 14))
        self.status_label.grid(row=self.board.rows, column=0, columnspan=self.board.cols)

        self.card_buttons_frame = tk.Frame(self.root)
        self.card_buttons_frame.grid(row=self.board.rows+1, column=0, columnspan=self.board.cols)

    def assign_fixed_labels(self):
        for i in range(self.board.rows):
            for j in range(self.board.cols):
                label = self.fixed_labels[i][j]
                self.board.label_map[i][j] = label
                if (i, j) in self.board.wild_positions:
                    self.buttons[i][j].config(text="SEQ", bg="gray")
                else:
                    self.buttons[i][j].config(text=label)

    def create_deck(self):
        labels = [label for row in self.fixed_labels for label in row if label != "SEQ"]
        random.shuffle(labels)
        self.deck = [Block(label) for label in labels]

    def start_game(self):
        for _ in range(2):
            self.human.hand.append(self.deck.pop())
            self.robot.hand.append(self.deck.pop())
        self.update_hand_buttons()
        self.status_label.config(text="Your turn! Select a card and click its tile.")

    def update_hand_buttons(self):
        for widget in self.card_buttons_frame.winfo_children():
            widget.destroy()
        for card in self.human.hand:
            btn = tk.Button(self.card_buttons_frame, text=card.card_label,
                            font=('Arial', 12), width=10,
                            command=lambda c=card: self.select_card(c))
            btn.pack(side='left', padx=10)

    def select_card(self, card):
        if card in self.human.hand:
            self.human.hand.remove(card)
            self.selected_card = card
            self.update_hand_buttons()
            self.status_label.config(text=f"Selected: {card.card_label}. Now click its matching tile.")

    def handle_click(self, i, j):
        if not self.selected_card:
            return
        if self.board.grid[i][j] != "":
            self.status_label.config(text="Tile already occupied.")
            return
        if (i, j) in self.board.wild_positions:
            self.status_label.config(text="Cannot place on SEQ tile.")
            return
        if self.board.label_map[i][j] != self.selected_card.card_label:
            self.status_label.config(text="Wrong tile.")
            return
        self.place_piece(i, j, self.human, self.selected_card.card_label)
        self.selected_card = None
        if check_win(self.board, self.human.symbol):
            self.status_label.config(text="You Win!")
            return
        if self.deck:
            self.human.hand.append(self.deck.pop())
        self.update_hand_buttons()
        self.turn += 1
        self.root.after(1000, self.robot_turn)


    def place_piece(self, i, j, player, label):
        self.board.grid[i][j] = player.symbol
        self.buttons[i][j].config(text=label, bg=player.color, state='disabled')

    def robot_turn(self):
        card = self.robot.hand.pop(0)
        placed = False

        for i in range(self.board.rows):
            for j in range(self.board.cols):
                if (i, j) not in self.board.wild_positions and \
                self.board.grid[i][j] == "" and \
                self.board.label_map[i][j] == card.card_label:
                    self.place_piece(i, j, self.robot, card.card_label)
                    placed = True
                    self.status_label.config(
                        text=f"Robot played {card.card_label} at {self.board.label_map[i][j]}"
                    )
                    break
            if placed:
                break

        if placed and check_win(self.board, self.robot.symbol):
            self.status_label.config(text="Robot Wins!")
            return

        if self.deck:
            self.robot.hand.append(self.deck.pop())

        self.turn += 1
        self.status_label.config(
            text=self.status_label.cget("text") + "\nYour turn! Select a card and click its tile.")
        

# Launch the game
if __name__ == "__main__":
    root = tk.Tk()
    app = SequenceGame(root)
    root.mainloop()
