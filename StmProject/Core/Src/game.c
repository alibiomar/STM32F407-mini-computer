/*
 * game.c
 *
 *  Created on: Oct 15, 2025
 *      Author: omara
 */


#include "game.h"
#include "main.h"
#include <stdlib.h>
#include <stdio.h>

// Game global variables
Game_t currentGame = {0};
bool gameMode = false;

void Game_Init(void) {
    currentGame.state = GAME_STATE_MENU;
    currentGame.currentGame = GAME_SNAKE;
    currentGame.score = 0;
    currentGame.highScore = 0;
    currentGame.gameSpeed = 200; // ms per update
    currentGame.exitConfirmed = false;
    currentGame.lastInputTime = 0;
}

void Game_Start(GameType_t game) {
    currentGame.currentGame = game;
    currentGame.state = GAME_STATE_PLAYING;
    currentGame.score = 0;
    currentGame.lastUpdateTime = HAL_GetTick();
    currentGame.exitConfirmed = false;

    switch(game) {
        case GAME_SNAKE:
            Snake_Init();
            break;
        case GAME_PONG:
            Pong_Init();
            break;
        case GAME_TETRIS:
            Tetris_Init();
            break;
        default:
            break;
    }

    gameMode = true;
}

void Game_Update(void) {
    if (currentGame.state != GAME_STATE_PLAYING) return;

    uint32_t currentTime = HAL_GetTick();
    if (currentTime - currentGame.lastUpdateTime < currentGame.gameSpeed) return;

    currentGame.lastUpdateTime = currentTime;

    switch(currentGame.currentGame) {
        case GAME_SNAKE:
            Snake_Update();
            break;
        case GAME_PONG:
            Pong_Update();
            break;
        case GAME_TETRIS:
            Tetris_Update();
            break;
        default:
            break;
    }
}

void Game_HandleInput(uint8_t button) {
    currentGame.lastInputTime = HAL_GetTick();

    // Button mapping (from main.c):
    // 1 = Exit (PC1)
    // 2 = Confirm/OK (PC2)
    // 3 = Up (PC3)
    // 4 = Down (PC4)

    // Exit button: leave game immediately (no long-press/confirm flow)
    if (button == 1) {
        Game_Exit();
        return;
    }

    // Exit-confirm state is kept for compatibility but no longer entered.
    if (currentGame.state == GAME_STATE_EXIT_CONFIRM) {
        currentGame.state = GAME_STATE_MENU;
        return;
    }

    // Handle pause menu
    if (currentGame.state == GAME_STATE_PAUSED) {
        if (button == 2) { // Confirm button - resume
            Game_Resume();
        }
        return;
    }

    // Handle game over state
    if (currentGame.state == GAME_STATE_GAME_OVER) {
        if (button == 2) { // Confirm button - restart
            Game_Start(currentGame.currentGame);
        }
        return;
    }

    // Handle main menu
    if (currentGame.state == GAME_STATE_MENU) {
        if (button == 3) { // Up - previous game
            currentGame.currentGame = (currentGame.currentGame + GAME_COUNT - 1) % GAME_COUNT;
        } else if (button == 4) { // Down - next game
            currentGame.currentGame = (currentGame.currentGame + 1) % GAME_COUNT;
        } else if (button == 2) { // Confirm button - select game
            Game_Start(currentGame.currentGame);
        }
        return;
    }

    // In-game controls
    if (currentGame.state == GAME_STATE_PLAYING) {
        // Game-specific input handling
        switch(currentGame.currentGame) {
            case GAME_SNAKE:
                if (button == 3) { // Up - turn left
                    currentGame.snake.direction = (currentGame.snake.direction + 3) % 4;
                } else if (button == 4) { // Down - turn right
                    currentGame.snake.direction = (currentGame.snake.direction + 1) % 4;
                } else if (button == 2) { // Confirm - speed boost
                    // Optional: can be used for speed boost
                }
                break;

            case GAME_PONG:
                if (button == 3) { // Up - paddle up
                    if (currentGame.leftPaddle.y > 2) currentGame.leftPaddle.y -= 4;
                } else if (button == 4) { // Down - paddle down
                    if (currentGame.leftPaddle.y < 54) currentGame.leftPaddle.y += 4;
                } else if (button == 2) { // Confirm - can be used for special action
                    // Optional: serve ball or boost
                }
                break;

            case GAME_TETRIS:
                if (button == 3) { // Up - move left
                    Tetris_MovePiece(-1);
                } else if (button == 4) { // Down - move right
                    Tetris_MovePiece(1);
                } else if (button == 2) { // Confirm - rotate
                    Tetris_RotatePiece();
                }
                break;

            default:
                break;
        }
    }
}
void Game_Render(void) {
    switch(currentGame.state) {
        case GAME_STATE_MENU:
            Game_Menu_Render();
            break;
        case GAME_STATE_EXIT_CONFIRM:
            Game_Exit_Confirm_Render();
            break;
        case GAME_STATE_PLAYING:
        case GAME_STATE_PAUSED:
        case GAME_STATE_GAME_OVER:
            switch(currentGame.currentGame) {
                case GAME_SNAKE:
                    Snake_Render();
                    break;
                case GAME_PONG:
                    Pong_Render();
                    break;
                case GAME_TETRIS:
                    Tetris_Render();
                    break;
                default:
                    break;
            }
            break;
    }
}

void Game_Pause(void) {
    if (currentGame.state == GAME_STATE_PLAYING) {
        currentGame.state = GAME_STATE_PAUSED;
    }
}

void Game_Resume(void) {
    if (currentGame.state == GAME_STATE_PAUSED) {
        currentGame.state = GAME_STATE_PLAYING;
        currentGame.lastUpdateTime = HAL_GetTick();
    }
}

void Game_End(void) {
    currentGame.state = GAME_STATE_MENU;
    gameMode = false;
}

// Snake Game Implementation
void Snake_Init(void) {
    // Initialize snake in the middle
    currentGame.snake.length = 3;
    currentGame.snake.direction = 1; // Start moving right

    for (uint8_t i = 0; i < currentGame.snake.length; i++) {
        currentGame.snake.segments[i].x = 32 - i * 2;
        currentGame.snake.segments[i].y = 16;
    }

    // Spawn first food
    currentGame.food.active = true;
    currentGame.food.position.x = 80;
    currentGame.food.position.y = 32;

    currentGame.gameSpeed = 200;
}

void Snake_Update(void) {
    // Move snake body
    for (uint8_t i = currentGame.snake.length - 1; i > 0; i--) {
        currentGame.snake.segments[i] = currentGame.snake.segments[i - 1];
    }

    // Move head based on direction
    Point_t newHead = currentGame.snake.segments[0];

    switch(currentGame.snake.direction) {
        case 0: // Up
            newHead.y -= 2;
            break;
        case 1: // Right
            newHead.x += 2;
            break;
        case 2: // Down
            newHead.y += 2;
            break;
        case 3: // Left
            newHead.x -= 2;
            break;
    }

    // Check wall collisions
    if (newHead.x < 2 || newHead.x > 125 || newHead.y < 2 || newHead.y > 61) {
        currentGame.state = GAME_STATE_GAME_OVER;
        return;
    }

    // Check self collision
    for (uint8_t i = 1; i < currentGame.snake.length; i++) {
        if (newHead.x == currentGame.snake.segments[i].x &&
            newHead.y == currentGame.snake.segments[i].y) {
            currentGame.state = GAME_STATE_GAME_OVER;
            return;
        }
    }

    currentGame.snake.segments[0] = newHead;

    // Check food collision
    if (currentGame.food.active &&
        newHead.x == currentGame.food.position.x &&
        newHead.y == currentGame.food.position.y) {

        // Grow snake
        if (currentGame.snake.length < 64) {
            currentGame.snake.length++;
        }

        // Increase score
        currentGame.score += 10;

        // Spawn new food at random position
        currentGame.food.position.x = 4 + (rand() % 60) * 2;
        currentGame.food.position.y = 4 + (rand() % 28) * 2;

        // Increase speed slightly
        if (currentGame.gameSpeed > 50) {
            currentGame.gameSpeed -= 5;
        }
    }
}
void Game_RequestExit(void) {
    if (currentGame.state == GAME_STATE_PAUSED || currentGame.state == GAME_STATE_PLAYING) {
        currentGame.state = GAME_STATE_EXIT_CONFIRM;
        currentGame.exitConfirmed = false;
    }
}


void Game_CancelExit(void) {
    if (currentGame.state == GAME_STATE_EXIT_CONFIRM) {
        // Return to previous state (either playing or paused)
        if (currentGame.lastInputTime > 0) {
            currentGame.state = GAME_STATE_PAUSED;
        } else {
            currentGame.state = GAME_STATE_PLAYING;
        }
    }
}
void Game_Exit(void) {
    currentGame.state = GAME_STATE_MENU;
    gameMode = false;
    currentGame.exitConfirmed = true;

    // Optional: Return to home screen instead of game menu
    // Uncomment the following lines if you want to exit completely to home:
    /*
    displayMode = DISPLAY_MODE_HOME;
    gameMode = false;
    */
}
void Game_Exit_Confirm_Render(void) {
    SH1106_Clear();

    // Draw border
    SH1106_DrawRectangle(0, 0, 128, 64, 1);

    // Title
    SH1106_GotoXY(22, 5);
    SH1106_Puts("EXIT (unused)", &Font_7x10, 1);

    // Message
    SH1106_GotoXY(10, 20);
    SH1106_Puts("Use Exit button", &Font_7x10, 1);
    SH1106_GotoXY(10, 30);
    SH1106_Puts("to quit game", &Font_7x10, 1);

    // Blinking confirmation indicator
    if ((HAL_GetTick() / 500) % 2) {
        SH1106_GotoXY(40, 45);
        SH1106_Puts("EXIT", &Font_7x10, 1);
    }
}
void Snake_Render(void) {
    SH1106_Clear();

    // Draw border
    SH1106_DrawRectangle(0, 0, 128, 64, 1);

    // Draw snake
    for (uint8_t i = 0; i < currentGame.snake.length; i++) {
        SH1106_DrawRectangle(
            currentGame.snake.segments[i].x,
            currentGame.snake.segments[i].y,
            2, 2, 1
        );
    }

    // Draw food
    if (currentGame.food.active) {
        SH1106_DrawRectangle(
            currentGame.food.position.x,
            currentGame.food.position.y,
            2, 2, 1
        );
    }

    // Draw score
    char scoreStr[16];
    snprintf(scoreStr, sizeof(scoreStr), "Score: %d", currentGame.score);
    SH1106_GotoXY(2, 2);
    SH1106_Puts(scoreStr, &Font_7x10, 1);

    // Draw game state
    if (currentGame.state == GAME_STATE_PAUSED) {
        SH1106_GotoXY(40, 25);
        SH1106_Puts("PAUSED", &Font_7x10, 1);

        // Show pause menu options
        SH1106_GotoXY(20, 40);
        SH1106_Puts("Confirm: Resume", &Font_7x10, 1);
        SH1106_GotoXY(20, 50);
        SH1106_Puts("Exit: Quit", &Font_7x10, 1);

    } else if (currentGame.state == GAME_STATE_GAME_OVER) {
        SH1106_GotoXY(30, 25);
        SH1106_Puts("GAME OVER", &Font_7x10, 1);

        // Show game over options
        SH1106_GotoXY(20, 40);
        SH1106_Puts("Confirm: Restart", &Font_7x10, 1);
        SH1106_GotoXY(20, 50);
        SH1106_Puts("Exit: Quit", &Font_7x10, 1);
    }

    // Show in-game controls hint (only when playing)
    if (currentGame.state == GAME_STATE_PLAYING) {
        SH1106_GotoXY(2, 55);
        SH1106_Puts("UP/DN=Turn  EXIT=Quit", &Font_7x10, 1);
    }
}
// Pong Game Implementation
void Pong_Init(void) {
    // Initialize paddles
    currentGame.leftPaddle.x = 4;
    currentGame.leftPaddle.y = 28;
    currentGame.leftPaddle.width = 2;
    currentGame.leftPaddle.height = 8;

    currentGame.rightPaddle.x = 122;
    currentGame.rightPaddle.y = 28;
    currentGame.rightPaddle.width = 2;
    currentGame.rightPaddle.height = 8;

    // Initialize ball
    currentGame.ball.x = 64;
    currentGame.ball.y = 32;
    currentGame.ball.dx = 1;
    currentGame.ball.dy = 1;

    currentGame.leftScore = 0;
    currentGame.rightScore = 0;

    currentGame.gameSpeed = 50;
}

void Pong_Update(void) {
    // Move ball
    currentGame.ball.x += currentGame.ball.dx;
    currentGame.ball.y += currentGame.ball.dy;

    // Ball collision with top/bottom
    if (currentGame.ball.y <= 2 || currentGame.ball.y >= 61) {
        currentGame.ball.dy = -currentGame.ball.dy;
    }

    // Ball collision with paddles
    if (currentGame.ball.x <= currentGame.leftPaddle.x + currentGame.leftPaddle.width &&
        currentGame.ball.x >= currentGame.leftPaddle.x &&
        currentGame.ball.y >= currentGame.leftPaddle.y &&
        currentGame.ball.y <= currentGame.leftPaddle.y + currentGame.leftPaddle.height) {
        currentGame.ball.dx = 1;
    }

    if (currentGame.ball.x >= currentGame.rightPaddle.x &&
        currentGame.ball.x <= currentGame.rightPaddle.x + currentGame.rightPaddle.width &&
        currentGame.ball.y >= currentGame.rightPaddle.y &&
        currentGame.ball.y <= currentGame.rightPaddle.y + currentGame.rightPaddle.height) {
        currentGame.ball.dx = -1;
    }

    // Score points
    if (currentGame.ball.x < 2) {
        currentGame.rightScore++;
        currentGame.ball.x = 64;
        currentGame.ball.y = 32;
        currentGame.ball.dx = 1;
    }

    if (currentGame.ball.x > 125) {
        currentGame.leftScore++;
        currentGame.ball.x = 64;
        currentGame.ball.y = 32;
        currentGame.ball.dx = -1;
    }

    // Beatable AI for right paddle:
    // - Only tracks the ball when it's moving toward the AI (dx > 0)
    // - Reaction delay + limited movement speed
    // - Deadzone around target
    // - Small, slowly-changing aiming error
    static uint32_t lastAiStepTime = 0;
    static uint32_t lastAiRecalcTime = 0;
    static int8_t aiError = 0;
    static int8_t aiTargetY = 28;

    const uint32_t now = HAL_GetTick();
    const uint32_t aiRecalcDelayMs = 120; // how often the AI updates its aim
    const uint32_t aiStepDelayMs = 55;    // how often the AI can move 1px
    const int8_t deadzone = 2;            // ignore tiny differences
    const int8_t minY = 2;
    const int8_t maxY = 64 - 2 - currentGame.rightPaddle.height;

    if ((now - lastAiRecalcTime) >= aiRecalcDelayMs) {
        lastAiRecalcTime = now;

        // Occasionally adjust error so the bot doesn't play perfectly
        // Keep it small so it still feels responsive.
        if ((rand() % 5) == 0) {
            aiError = (int8_t)((rand() % 3) - 1); // [-1..+1]
        }

        if (currentGame.ball.dx > 0) {
            // Aim for ball center with some error
            int16_t desired = (int16_t)currentGame.ball.y - (currentGame.rightPaddle.height / 2) + aiError;
            if (desired < minY) desired = minY;
            if (desired > maxY) desired = maxY;
            aiTargetY = (int8_t)desired;
        } else {
            // When ball is moving away, drift toward center
            aiTargetY = 28;
        }
    }

    if ((now - lastAiStepTime) >= aiStepDelayMs) {
        lastAiStepTime = now;
        int16_t diff = (int16_t)aiTargetY - (int16_t)currentGame.rightPaddle.y;
        if (diff > deadzone) {
            if (currentGame.rightPaddle.y < maxY) currentGame.rightPaddle.y += 1;
        } else if (diff < -deadzone) {
            if (currentGame.rightPaddle.y > minY) currentGame.rightPaddle.y -= 1;
        }
    }
}

void Pong_Render(void) {
    SH1106_Clear();

    // Draw center line
    for (uint8_t y = 2; y < 62; y += 4) {
        SH1106_DrawRectangle(63, y, 2, 2, 1);
    }

    // Draw paddles
    SH1106_Fill_Rectangle(
        currentGame.leftPaddle.x,
        currentGame.leftPaddle.y,
        currentGame.leftPaddle.width,
        currentGame.leftPaddle.height,
        1
    );

    SH1106_Fill_Rectangle(
        currentGame.rightPaddle.x,
        currentGame.rightPaddle.y,
        currentGame.rightPaddle.width,
        currentGame.rightPaddle.height,
        1
    );

    // Draw ball
    SH1106_DrawRectangle(currentGame.ball.x, currentGame.ball.y, 2, 2, 1);

    // Draw scores
    char scoreStr[8];
    snprintf(scoreStr, sizeof(scoreStr), "%d", currentGame.leftScore);
    SH1106_GotoXY(50, 2);
    SH1106_Puts(scoreStr, &Font_7x10, 1);

    snprintf(scoreStr, sizeof(scoreStr), "%d", currentGame.rightScore);
    SH1106_GotoXY(70, 2);
    SH1106_Puts(scoreStr, &Font_7x10, 1);

    // Draw game state
    if (currentGame.state == GAME_STATE_PAUSED) {
        SH1106_GotoXY(40, 25);
        SH1106_Puts("PAUSED", &Font_7x10, 1);

        SH1106_GotoXY(20, 40);
        SH1106_Puts("Confirm: Resume", &Font_7x10, 1);
        SH1106_GotoXY(20, 50);
        SH1106_Puts("Exit: Quit", &Font_7x10, 1);

    } else if (currentGame.state == GAME_STATE_GAME_OVER) {
        SH1106_GotoXY(30, 25);
        SH1106_Puts("GAME OVER", &Font_7x10, 1);

        SH1106_GotoXY(20, 40);
        SH1106_Puts("Confirm: Restart", &Font_7x10, 1);
        SH1106_GotoXY(20, 50);
        SH1106_Puts("Exit: Quit", &Font_7x10, 1);
    }

    // Show in-game controls hint
    if (currentGame.state == GAME_STATE_PLAYING) {
        SH1106_GotoXY(10, 55);
        SH1106_Puts("UP/DN=Move  EXIT=Quit", &Font_7x10, 1);
    }
}
// Tetris Game Implementation
// Tetromino shapes [piece][rotation][y][x]
const int8_t tetrisShapes[7][4][4][4] = {
    // I piece
    {
        {{0,0,0,0},{1,1,1,1},{0,0,0,0},{0,0,0,0}},
        {{0,0,1,0},{0,0,1,0},{0,0,1,0},{0,0,1,0}},
        {{0,0,0,0},{0,0,0,0},{1,1,1,1},{0,0,0,0}},
        {{0,1,0,0},{0,1,0,0},{0,1,0,0},{0,1,0,0}}
    },
    // O piece
    {
        {{0,1,1,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,1,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,1,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,1,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}}
    },
    // T piece
    {
        {{0,1,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,0,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,1,0},{0,1,0,0},{0,0,0,0}},
        {{0,1,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}}
    },
    // S piece
    {
        {{0,1,1,0},{1,1,0,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,0,0},{0,1,1,0},{0,0,1,0},{0,0,0,0}},
        {{0,0,0,0},{0,1,1,0},{1,1,0,0},{0,0,0,0}},
        {{1,0,0,0},{1,1,0,0},{0,1,0,0},{0,0,0,0}}
    },
    // Z piece
    {
        {{1,1,0,0},{0,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,0,1,0},{0,1,1,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,0,0},{0,1,1,0},{0,0,0,0}},
        {{0,1,0,0},{1,1,0,0},{1,0,0,0},{0,0,0,0}}
    },
    // J piece
    {
        {{1,0,0,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,1,0},{0,1,0,0},{0,1,0,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,1,0},{0,0,1,0},{0,0,0,0}},
        {{0,1,0,0},{0,1,0,0},{1,1,0,0},{0,0,0,0}}
    },
    // L piece
    {
        {{0,0,1,0},{1,1,1,0},{0,0,0,0},{0,0,0,0}},
        {{0,1,0,0},{0,1,0,0},{0,1,1,0},{0,0,0,0}},
        {{0,0,0,0},{1,1,1,0},{1,0,0,0},{0,0,0,0}},
        {{1,1,0,0},{0,1,0,0},{0,1,0,0},{0,0,0,0}}
    }
};

void Tetris_Init(void) {
    // Clear the grid
    for (uint8_t y = 0; y < TETRIS_HEIGHT; y++) {
        for (uint8_t x = 0; x < TETRIS_WIDTH; x++) {
            currentGame.tetris.grid[y][x] = 0;
        }
    }

    // Initialize game state
    currentGame.tetris.linesCleared = 0;
    currentGame.tetris.lastDropTime = HAL_GetTick();
    currentGame.tetris.dropDelay = 800; // Start at 800ms per drop
    currentGame.tetris.fastDrop = false;
    currentGame.score = 0;
    currentGame.gameSpeed = 800;

    // Spawn first piece
    Tetris_SpawnPiece();
}

void Tetris_SpawnPiece(void) {
    // Set current piece to next piece if exists, otherwise random
    if (currentGame.tetris.nextPiece.active) {
        currentGame.tetris.currentPiece = currentGame.tetris.nextPiece;
    } else {
        currentGame.tetris.currentPiece.type = rand() % PIECE_COUNT;
    }
    
    currentGame.tetris.currentPiece.x = TETRIS_WIDTH / 2 - 2;
    currentGame.tetris.currentPiece.y = 0;
    currentGame.tetris.currentPiece.rotation = 0;
    currentGame.tetris.currentPiece.active = true;

    // Generate next piece
    currentGame.tetris.nextPiece.type = rand() % PIECE_COUNT;
    currentGame.tetris.nextPiece.active = true;

    // Check if spawn position is blocked (game over)
    if (Tetris_CheckCollision(0, 0, currentGame.tetris.currentPiece.rotation)) {
        currentGame.state = GAME_STATE_GAME_OVER;
    }
}

bool Tetris_CheckCollision(int8_t offsetX, int8_t offsetY, uint8_t rotation) {
    TetrisCurrentPiece_t* piece = &currentGame.tetris.currentPiece;
    
    for (int8_t y = 0; y < 4; y++) {
        for (int8_t x = 0; x < 4; x++) {
            if (tetrisShapes[piece->type][rotation][y][x]) {
                int8_t newX = piece->x + x + offsetX;
                int8_t newY = piece->y + y + offsetY;

                // Check boundaries
                if (newX < 0 || newX >= TETRIS_WIDTH || newY >= TETRIS_HEIGHT) {
                    return true;
                }

                // Check collision with locked pieces (ignore if above grid)
                if (newY >= 0 && currentGame.tetris.grid[newY][newX]) {
                    return true;
                }
            }
        }
    }
    return false;
}

void Tetris_MovePiece(int8_t dx) {
    if (!currentGame.tetris.currentPiece.active) return;
    
    if (!Tetris_CheckCollision(dx, 0, currentGame.tetris.currentPiece.rotation)) {
        currentGame.tetris.currentPiece.x += dx;
    }
}

void Tetris_RotatePiece(void) {
    if (!currentGame.tetris.currentPiece.active) return;
    
    uint8_t newRotation = (currentGame.tetris.currentPiece.rotation + 1) % 4;
    
    // Try rotation
    if (!Tetris_CheckCollision(0, 0, newRotation)) {
        currentGame.tetris.currentPiece.rotation = newRotation;
    }
    // Try wall kick (move left/right if rotation blocked)
    else if (!Tetris_CheckCollision(-1, 0, newRotation)) {
        currentGame.tetris.currentPiece.x -= 1;
        currentGame.tetris.currentPiece.rotation = newRotation;
    }
    else if (!Tetris_CheckCollision(1, 0, newRotation)) {
        currentGame.tetris.currentPiece.x += 1;
        currentGame.tetris.currentPiece.rotation = newRotation;
    }
}

void Tetris_DropPiece(void) {
    if (!currentGame.tetris.currentPiece.active) return;
    
    if (!Tetris_CheckCollision(0, 1, currentGame.tetris.currentPiece.rotation)) {
        currentGame.tetris.currentPiece.y++;
    } else {
        // Lock the piece
        Tetris_LockPiece();
    }
}

void Tetris_LockPiece(void) {
    TetrisCurrentPiece_t* piece = &currentGame.tetris.currentPiece;
    
    // Add piece to grid
    for (int8_t y = 0; y < 4; y++) {
        for (int8_t x = 0; x < 4; x++) {
            if (tetrisShapes[piece->type][piece->rotation][y][x]) {
                int8_t gridY = piece->y + y;
                int8_t gridX = piece->x + x;
                
                if (gridY >= 0 && gridY < TETRIS_HEIGHT && gridX >= 0 && gridX < TETRIS_WIDTH) {
                    currentGame.tetris.grid[gridY][gridX] = piece->type + 1;
                }
            }
        }
    }

    piece->active = false;
    
    // Check for completed lines
    Tetris_ClearLines();
    
    // Spawn new piece
    Tetris_SpawnPiece();
}

void Tetris_ClearLines(void) {
    uint8_t linesCleared = 0;
    
    // Check each row from bottom to top
    for (int8_t y = TETRIS_HEIGHT - 1; y >= 0; y--) {
        bool lineComplete = true;
        
        for (uint8_t x = 0; x < TETRIS_WIDTH; x++) {
            if (currentGame.tetris.grid[y][x] == 0) {
                lineComplete = false;
                break;
            }
        }
        
        if (lineComplete) {
            linesCleared++;
            
            // Move all rows above down
            for (int8_t moveY = y; moveY > 0; moveY--) {
                for (uint8_t x = 0; x < TETRIS_WIDTH; x++) {
                    currentGame.tetris.grid[moveY][x] = currentGame.tetris.grid[moveY - 1][x];
                }
            }
            
            // Clear top row
            for (uint8_t x = 0; x < TETRIS_WIDTH; x++) {
                currentGame.tetris.grid[0][x] = 0;
            }
            
            y++; // Check this row again
        }
    }
    
    if (linesCleared > 0) {
        currentGame.tetris.linesCleared += linesCleared;
        
        // Score: 1 line=100, 2=300, 3=500, 4=800
        uint16_t lineScore[] = {0, 100, 300, 500, 800};
        currentGame.score += (linesCleared <= 4) ? lineScore[linesCleared] : 800;
        
        // Increase speed every 10 lines
        if (currentGame.tetris.linesCleared % 10 == 0 && currentGame.tetris.dropDelay > 100) {
            currentGame.tetris.dropDelay -= 50;
            currentGame.gameSpeed = currentGame.tetris.dropDelay;
        }
    }
}

void Tetris_Update(void) {
    if (!currentGame.tetris.currentPiece.active) {
        Tetris_SpawnPiece();
        return;
    }

    uint32_t currentTime = HAL_GetTick();
    uint32_t dropDelay = currentGame.tetris.fastDrop ? 50 : currentGame.tetris.dropDelay;
    
    if (currentTime - currentGame.tetris.lastDropTime >= dropDelay) {
        Tetris_DropPiece();
        currentGame.tetris.lastDropTime = currentTime;
        currentGame.tetris.fastDrop = false;
    }
}

void Tetris_Render(void) {
    SH1106_Clear();

    // Calculate drawing offsets
    uint8_t gridStartX = 10;
    uint8_t gridStartY = 2;

    // Draw border around game area
    SH1106_DrawRectangle(
        gridStartX - 1, 
        gridStartY - 1, 
        TETRIS_WIDTH * TETRIS_BLOCK_SIZE + 2, 
        TETRIS_HEIGHT * TETRIS_BLOCK_SIZE + 2, 
        1
    );

    // Draw locked pieces
    for (uint8_t y = 0; y < TETRIS_HEIGHT; y++) {
        for (uint8_t x = 0; x < TETRIS_WIDTH; x++) {
            if (currentGame.tetris.grid[y][x]) {
                uint8_t drawX = gridStartX + x * TETRIS_BLOCK_SIZE;
                uint8_t drawY = gridStartY + y * TETRIS_BLOCK_SIZE;
                SH1106_Fill_Rectangle(drawX, drawY, TETRIS_BLOCK_SIZE - 1, TETRIS_BLOCK_SIZE - 1, 1);
            }
        }
    }

    // Draw current piece
    if (currentGame.tetris.currentPiece.active) {
        TetrisCurrentPiece_t* piece = &currentGame.tetris.currentPiece;
        
        for (int8_t y = 0; y < 4; y++) {
            for (int8_t x = 0; x < 4; x++) {
                if (tetrisShapes[piece->type][piece->rotation][y][x]) {
                    int8_t drawY = piece->y + y;
                    int8_t drawX = piece->x + x;
                    
                    if (drawY >= 0 && drawY < TETRIS_HEIGHT && drawX >= 0 && drawX < TETRIS_WIDTH) {
                        uint8_t screenX = gridStartX + drawX * TETRIS_BLOCK_SIZE;
                        uint8_t screenY = gridStartY + drawY * TETRIS_BLOCK_SIZE;
                        SH1106_Fill_Rectangle(screenX, screenY, TETRIS_BLOCK_SIZE - 1, TETRIS_BLOCK_SIZE - 1, 1);
                    }
                }
            }
        }
    }

    // Draw score and lines on the right side
    char scoreStr[16];
    snprintf(scoreStr, sizeof(scoreStr), "S:%d", currentGame.score);
    SH1106_GotoXY(gridStartX + TETRIS_WIDTH * TETRIS_BLOCK_SIZE + 5, 5);
    SH1106_Puts(scoreStr, &Font_7x10, 1);

    snprintf(scoreStr, sizeof(scoreStr), "L:%d", currentGame.tetris.linesCleared);
    SH1106_GotoXY(gridStartX + TETRIS_WIDTH * TETRIS_BLOCK_SIZE + 5, 15);
    SH1106_Puts(scoreStr, &Font_7x10, 1);

    // Draw next piece preview
    SH1106_GotoXY(gridStartX + TETRIS_WIDTH * TETRIS_BLOCK_SIZE + 5, 25);
    SH1106_Puts("Next:", &Font_7x10, 1);
    
    if (currentGame.tetris.nextPiece.active) {
        uint8_t nextX = gridStartX + TETRIS_WIDTH * TETRIS_BLOCK_SIZE + 5;
        uint8_t nextY = 35;
        
        for (int8_t y = 0; y < 4; y++) {
            for (int8_t x = 0; x < 4; x++) {
                if (tetrisShapes[currentGame.tetris.nextPiece.type][0][y][x]) {
                    SH1106_Fill_Rectangle(
                        nextX + x * TETRIS_BLOCK_SIZE, 
                        nextY + y * TETRIS_BLOCK_SIZE, 
                        TETRIS_BLOCK_SIZE - 1, 
                        TETRIS_BLOCK_SIZE - 1, 
                        1
                    );
                }
            }
        }
    }

    // Draw game state overlays
    if (currentGame.state == GAME_STATE_PAUSED) {
        SH1106_Fill_Rectangle(30, 20, 68, 25, 0);
        SH1106_DrawRectangle(30, 20, 68, 25, 1);
        SH1106_GotoXY(40, 23);
        SH1106_Puts("PAUSED", &Font_7x10, 1);
        SH1106_GotoXY(35, 33);
        SH1106_Puts("Exit=Quit", &Font_7x10, 1);

    } else if (currentGame.state == GAME_STATE_GAME_OVER) {
        SH1106_Fill_Rectangle(20, 20, 88, 25, 0);
        SH1106_DrawRectangle(20, 20, 88, 25, 1);
        SH1106_GotoXY(30, 23);
        SH1106_Puts("GAME OVER", &Font_7x10, 1);
        SH1106_GotoXY(25, 33);
        SH1106_Puts("Confirm=Retry", &Font_7x10, 1);
    }
}

// Game Menu
void Game_Menu_Render(void) {
    SH1106_Clear();

    SH1106_GotoXY(40, 2);
    SH1106_Puts("GAMES", &Font_7x10, 1);

    // Draw game options
    const char* gameNames[] = {"Snake", "Pong", "Tetris"};

    for (uint8_t i = 0; i < GAME_COUNT; i++) {
        uint8_t yPos = 15 + i * 15;

        if (i == currentGame.currentGame) {
            // Selected item
            SH1106_DrawRectangle(20, yPos - 2, 88, 12, 1);
            SH1106_GotoXY(40, yPos);
            SH1106_Puts(gameNames[i], &Font_7x10, 0);
        } else {
            // Normal item
            SH1106_GotoXY(40, yPos);
            SH1106_Puts(gameNames[i], &Font_7x10, 1);
        }
    }

    // Instructions
    SH1106_GotoXY(10, 55);
    SH1106_Puts("UP/DN Nav, OK Sel", &Font_7x10, 1);
}
