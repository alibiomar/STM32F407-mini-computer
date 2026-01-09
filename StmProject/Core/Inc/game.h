/*
 * game.h
 *
 *  Created on: Oct 15, 2025
 *      Author: omara
 */

#ifndef INC_GAME_H_
#define INC_GAME_H_


#include "main.h"
#include "SH1106.h"
#include "fonts.h"
#include <stdbool.h>

// Game types
typedef enum {
    GAME_SNAKE,
    GAME_PONG,
    GAME_TETRIS,
    GAME_COUNT
} GameType_t;

// Game states
typedef enum {
    GAME_STATE_MENU,
    GAME_STATE_PLAYING,
    GAME_STATE_PAUSED,
    GAME_STATE_GAME_OVER,
    GAME_STATE_EXIT_CONFIRM  // Add exit confirmation state
} GameState_t;

// Snake game structures
typedef struct {
    int8_t x;
    int8_t y;
} Point_t;

typedef struct {
    Point_t segments[64];
    uint8_t length;
    int8_t direction; // 0=up, 1=right, 2=down, 3=left
    uint32_t lastMoveTime;
} Snake_t;

typedef struct {
    Point_t position;
    bool active;
} Food_t;

// Pong game structures
typedef struct {
    int8_t x;
    int8_t y;
    int8_t width;
    int8_t height;
} Paddle_t;

typedef struct {
    int8_t x;
    int8_t y;
    int8_t dx;
    int8_t dy;
} Ball_t;

// Tetris game structures
#define TETRIS_WIDTH 10
#define TETRIS_HEIGHT 18
#define TETRIS_BLOCK_SIZE 3

typedef enum {
    PIECE_I,
    PIECE_O,
    PIECE_T,
    PIECE_S,
    PIECE_Z,
    PIECE_J,
    PIECE_L,
    PIECE_COUNT
} TetrisPiece_t;

typedef struct {
    int8_t x;
    int8_t y;
    TetrisPiece_t type;
    uint8_t rotation; // 0-3
    bool active;
} TetrisCurrentPiece_t;

typedef struct {
    uint8_t grid[TETRIS_HEIGHT][TETRIS_WIDTH];
    TetrisCurrentPiece_t currentPiece;
    TetrisCurrentPiece_t nextPiece;
    uint16_t linesCleared;
    uint32_t lastDropTime;
    uint32_t dropDelay;
    bool fastDrop;
} TetrisGame_t;

// Main game structure
typedef struct {
    GameType_t currentGame;
    GameState_t state;
    uint16_t score;
    uint16_t highScore;

    // Snake game
    Snake_t snake;
    Food_t food;

    // Pong game
    Paddle_t leftPaddle;
    Paddle_t rightPaddle;
    Ball_t ball;
    uint8_t leftScore;
    uint8_t rightScore;

    // Tetris game
    TetrisGame_t tetris;

    // Timing
    uint32_t lastUpdateTime;
    uint16_t gameSpeed;

    // Exit confirmation
    bool exitConfirmed;
    uint32_t lastInputTime;
} Game_t;

// Function prototypes
void Game_Init(void);
void Game_Start(GameType_t game);
void Game_Update(void);
void Game_HandleInput(uint8_t button);
void Game_Render(void);
void Game_Pause(void);
void Game_Resume(void);
void Game_Exit(void);
void Game_RequestExit(void);
void Game_CancelExit(void);

// Individual game functions
void Snake_Init(void);
void Snake_Update(void);
void Snake_Render(void);
bool Snake_CheckCollision(void);

void Pong_Init(void);
void Pong_Update(void);
void Pong_Render(void);

void Tetris_Init(void);
void Tetris_Update(void);
void Tetris_Render(void);
bool Tetris_CheckCollision(int8_t offsetX, int8_t offsetY, uint8_t rotation);
void Tetris_SpawnPiece(void);
void Tetris_LockPiece(void);
void Tetris_ClearLines(void);
void Tetris_RotatePiece(void);
void Tetris_MovePiece(int8_t dx);
void Tetris_DropPiece(void);

void Game_Menu_Render(void);
void Game_Exit_Confirm_Render(void);

// External variables
extern Game_t currentGame;
extern bool gameMode;


#endif /* INC_GAME_H_ */
