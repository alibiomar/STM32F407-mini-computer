/*
 * game.c
 *
 *  Created on: Oct 15, 2025
 *      Author: omara
 */


#include "game.h"
#include "main.h"
#include <stdlib.h>

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

    // Handle exit confirmation first
    if (currentGame.state == GAME_STATE_EXIT_CONFIRM) {
        if (button == 2) { // Confirm button - exit game
            Game_Exit();
        } else if (button == 1) { // Switch button - cancel exit
            Game_CancelExit();
        }
        return;
    }

    // Handle pause menu
    if (currentGame.state == GAME_STATE_PAUSED) {
        if (button == 2) { // Confirm button - resume
            Game_Resume();
        } else if (button == 1) { // Switch button - exit confirmation
            Game_RequestExit();
        }
        return;
    }

    // Handle game over state
    if (currentGame.state == GAME_STATE_GAME_OVER) {
        if (button == 2) { // Confirm button - restart
            Game_Start(currentGame.currentGame);
        } else if (button == 1) { // Switch button - exit to menu
            currentGame.state = GAME_STATE_MENU;
        }
        return;
    }

    // Handle main menu
    if (currentGame.state == GAME_STATE_MENU) {
        if (button == 1) { // Switch button - navigate
            currentGame.currentGame = (currentGame.currentGame + 1) % GAME_COUNT;
        } else if (button == 2) { // Confirm button - select
            Game_Start(currentGame.currentGame);
        }
        return;
    }

    // In-game controls with exit functionality
    if (currentGame.state == GAME_STATE_PLAYING) {
        // Long press detection for exit (hold switch button for 1 second)
        static uint32_t switchPressTime = 0;
        static bool switchPressed = false;

        if (button == 1) { // Switch button
            if (!switchPressed) {
                switchPressTime = HAL_GetTick();
                switchPressed = true;
            } else {
                // Check for long press (1 second)
                if (HAL_GetTick() - switchPressTime > 3000) {
                    Game_Pause();
                    switchPressed = false;
                    return;
                }
            }
        } else {
            switchPressed = false;
        }

        // Game-specific input handling
        switch(currentGame.currentGame) {
            case GAME_SNAKE:
                if (button == 1) { // Switch - change direction left
                    currentGame.snake.direction = (currentGame.snake.direction + 3) % 4;
                } else if (button == 2) { // Confirm - change direction right
                    currentGame.snake.direction = (currentGame.snake.direction + 1) % 4;
                }
                break;

            case GAME_PONG:
                if (button == 1) { // Switch - left paddle up
                    if (currentGame.leftPaddle.y > 2) currentGame.leftPaddle.y -= 4; // Move 2 pixels instead of 1
                } else if (button == 2) { // Confirm - left paddle down
                    if (currentGame.leftPaddle.y < 54) currentGame.leftPaddle.y += 4; // Move 2 pixels instead of 1
                }
                break;

            case GAME_TETRIS:
                if (button == 1) { // Switch - move left/rotate
                    // Implementation for Tetris
                } else if (button == 2) { // Confirm - move right/drop
                    // Implementation for Tetris
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
    SH1106_GotoXY(30, 5);
    SH1106_Puts("EXIT GAME?", &Font_7x10, 1);

    // Message
    SH1106_GotoXY(15, 20);
    SH1106_Puts("Confirm: Exit", &Font_7x10, 1);
    SH1106_GotoXY(15, 30);
    SH1106_Puts("Switch: Cancel", &Font_7x10, 1);

    // Blinking confirmation indicator
    if ((HAL_GetTick() / 500) % 2) {
        SH1106_GotoXY(45, 45);
        SH1106_Puts("EXIT?", &Font_7x10, 1);
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
        SH1106_Puts("Switch: Exit", &Font_7x10, 1);

    } else if (currentGame.state == GAME_STATE_GAME_OVER) {
        SH1106_GotoXY(30, 25);
        SH1106_Puts("GAME OVER", &Font_7x10, 1);

        // Show game over options
        SH1106_GotoXY(20, 40);
        SH1106_Puts("Confirm: Restart", &Font_7x10, 1);
        SH1106_GotoXY(20, 50);
        SH1106_Puts("Switch: Menu", &Font_7x10, 1);
    }

    // Show in-game exit hint (only when playing)
    if (currentGame.state == GAME_STATE_PLAYING) {
        SH1106_GotoXY(2, 55);
        SH1106_Puts("Hold Switch=Exit", &Font_7x10, 1);
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

    // Simple AI for right paddle
    if (currentGame.ball.y > currentGame.rightPaddle.y + currentGame.rightPaddle.height / 2) {
        if (currentGame.rightPaddle.y < 54) currentGame.rightPaddle.y++;
    } else if (currentGame.ball.y < currentGame.rightPaddle.y + currentGame.rightPaddle.height / 2) {
        if (currentGame.rightPaddle.y > 2) currentGame.rightPaddle.y--;
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
        SH1106_Puts("Switch: Exit", &Font_7x10, 1);

    } else if (currentGame.state == GAME_STATE_GAME_OVER) {
        SH1106_GotoXY(30, 25);
        SH1106_Puts("GAME OVER", &Font_7x10, 1);

        SH1106_GotoXY(20, 40);
        SH1106_Puts("Confirm: Restart", &Font_7x10, 1);
        SH1106_GotoXY(20, 50);
        SH1106_Puts("Switch: Menu", &Font_7x10, 1);
    }

    // Show in-game exit hint
    if (currentGame.state == GAME_STATE_PLAYING) {
        SH1106_GotoXY(2, 55);
        SH1106_Puts("Hold Switch=Exit", &Font_7x10, 1);
    }
}
// Tetris Game Implementation (simplified)
void Tetris_Init(void) {
    // Basic Tetris initialization
    currentGame.score = 0;
    currentGame.gameSpeed = 500;
}

void Tetris_Update(void) {
    // Basic Tetris update logic
    // This would need a full implementation
}

void Tetris_Render(void) {
    SH1106_Clear();
    SH1106_GotoXY(40, 25);
    SH1106_Puts("TETRIS", &Font_7x10, 1);
    SH1106_GotoXY(20, 40);
    SH1106_Puts("Coming Soon!", &Font_7x10, 1);
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
    SH1106_GotoXY(15, 55);
    SH1106_Puts("Switch Confirm", &Font_7x10, 1);
}
