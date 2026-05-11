#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "embdC_quat.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

// Global font for better readability
Font mainFont;

// Helper to draw text using our high-quality font
void MyDrawText(const char* text, float x, float y, float fontSize, Color color) {
    DrawTextEx(mainFont, text, {x, y}, fontSize, 1.0f, color);
}

// Helper to convert embdC_quat types to Raylib types
Vector3 ToRayVec(vec3_t v) { return {v.x, v.y, v.z}; }
Quaternion ToRayQuat(quat_t q) { return {q.x, q.y, q.z, q.w}; }

// Helper to draw text in 3D space (projected to 2D screen)
void DrawText3D(const char* text, Vector3 position, float fontSize, Color color, Camera3D camera) {
    Vector2 screenPos = GetWorldToScreen(position, camera);
    DrawTextEx(mainFont, text, screenPos, fontSize, 1.0f, color);
}

void SaveConfig(vec3_t pos, euler_t euler) {
    std::ofstream file("imu_config.txt");
    if (file.is_open()) {
        file << pos.x << " " << pos.y << " " << pos.z << "\n";
        file << euler.roll << " " << euler.pitch << " " << euler.yaw << "\n";

        quat_t q = quat_from_euler(euler);
        vec3_t c0 = quat_rotate_vec3(q, {1, 0, 0});
        vec3_t c1 = quat_rotate_vec3(q, {0, 1, 0});
        vec3_t c2 = quat_rotate_vec3(q, {0, 0, 1});

        file << "\n# Sensor-to-Body Rotation Matrix (R_s2b):\n";
        file << "# Row 1: " << c0.x << " " << c1.x << " " << c2.x << "\n";
        file << "# Row 2: " << c0.y << " " << c1.y << " " << c2.y << "\n";
        file << "# Row 3: " << c0.z << " " << c1.z << " " << c2.z << "\n";

        file.close();
    }
}

bool LoadConfig(vec3_t &pos, euler_t &euler) {
    std::ifstream file("imu_config.txt");
    if (file.is_open()) {
        if (!(file >> pos.x >> pos.y >> pos.z)) return false;
        if (!(file >> euler.roll >> euler.pitch >> euler.yaw)) return false;
        file.close();
        return true;
    }
    return false;
}

// Simple Input Box implementation
struct InputBox {
    Rectangle rect;
    std::string text;
    std::string label;
    bool active;
    int maxChars;

    void Update() {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            active = CheckCollisionPointRec(GetMousePosition(), rect);
        }

        if (active) {
            int key = GetCharPressed();
            while (key > 0) {
                if ((key >= 32) && (key <= 125) && (text.length() < maxChars)) {
                    text += (char)key;
                }
                key = GetCharPressed();
            }

            if (IsKeyPressed(KEY_BACKSPACE) && !text.empty()) {
                text.pop_back();
            }
        }
    }

    void Draw() {
        DrawRectangleRec(rect, active ? LIGHTGRAY : WHITE);
        DrawRectangleLinesEx(rect, 1, active ? BLUE : DARKGRAY);
        MyDrawText(label.c_str(), rect.x, rect.y - 18, 16, BLACK);
        MyDrawText(text.c_str(), rect.x + 5, rect.y + 5, 20, BLACK);
        if (active && (int)(GetTime() * 2) % 2 == 0) {
            float width = MeasureTextEx(mainFont, text.c_str(), 20, 1.0f).x;
            MyDrawText("|", rect.x + 8 + width, rect.y + 5, 20, BLACK);
        }
    }

    float GetValue() {
        try { return std::stof(text); } catch (...) { return 0.0f; }
    }

    void SetValue(float val) {
        std::stringstream ss;
        ss.precision(2);
        ss << std::fixed << val;
        text = ss.str();
    }
};

/**
 * GRAPHICS & UI CONFIGURATION
 * Adjust these values to change the simulation resolution and appearance.
 */
struct GraphicsConfig {
    int renderWidth  = 1280; // Width of the 3D simulation area
    int renderHeight = 720;  // Height of the simulation (720p)
    int sidebarWidth = 400;  // Width of the controls sidebar
    int msaaSamples  = 4;    // Anti-aliasing (0, 2, 4, 8). Fixes "blocky" edges.
    int targetFPS    = 60;   // Frame rate limit
} config;

/**
 * CONTROL CONFIGURATION
 * Define all keyboard mappings here for easy adjustment.
 */
struct ControlConfig {
    // Body Rotation
    int bodyPitchUp    = KEY_W;
    int bodyPitchDown  = KEY_S;
    int bodyYawLeft    = KEY_A;
    int bodyYawRight   = KEY_D;
    int bodyRollLeft   = KEY_Q;
    int bodyRollRight  = KEY_E;

    // IMU Position
    int imuPosXInc     = KEY_I;
    int imuPosXDec     = KEY_K;
    int imuPosYInc     = KEY_J;
    int imuPosYDec     = KEY_L;
    int imuPosZInc     = KEY_U;
    int imuPosZDec     = KEY_O;

    // IMU Orientation
    int imuPitchInc    = KEY_T;
    int imuPitchDec    = KEY_G;
    int imuYawLeft     = KEY_F;
    int imuYawRight    = KEY_H;
    int imuRollLeft    = KEY_V;
    int imuRollRight   = KEY_B;

    // System
    int saveConfig     = KEY_P;
    int loadConfig     = KEY_C;
    int resetAll       = KEY_R;
} keys;

int main() {
    const int screenWidth = config.renderWidth + config.sidebarWidth;
    const int screenHeight = config.renderHeight;

    // Enable Anti-Aliasing (MSAA) before window initialization
    if (config.msaaSamples > 0) {
        SetConfigFlags(FLAG_MSAA_4X_HINT); 
    }

    InitWindow(screenWidth, screenHeight, "EmbdC Quaternions - Sidebar Config");

    // Load High Quality Font
    mainFont = LoadFontEx("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf", 32, 0, 250);
    SetTextureFilter(mainFont.texture, TEXTURE_FILTER_BILINEAR);

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 0.0f, 1.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(config.targetFPS);

    euler_t current_euler = {0.0f, 0.0f, 0.0f};
    vec3_t imu_pos = {0.0f, 0.0f, 0.5f};
    euler_t imu_euler = {0.0f, 0.0f, 0.0f};

    LoadConfig(imu_pos, imu_euler);

    // Initialize Input Boxes
    std::vector<InputBox> inputs;
    float startX = config.renderWidth + 40;
    float startY = 280; 
    float spacing = 55;  

    const char* labels[] = {"Pos X", "Pos Y", "Pos Z", "Roll [deg]", "Pitch [deg]", "Yaw [deg]"};
    for (int i = 0; i < 6; i++) {
        inputs.push_back({{startX, startY + i * spacing, 140, 35}, "", labels[i], false, 10});
    }

    inputs[0].SetValue(imu_pos.x); inputs[1].SetValue(imu_pos.y); inputs[2].SetValue(imu_pos.z);
    inputs[3].SetValue(imu_euler.roll * RAD2DEG); inputs[4].SetValue(imu_euler.pitch * RAD2DEG); inputs[5].SetValue(imu_euler.yaw * RAD2DEG);

    while (!WindowShouldClose()) {
        // Update inputs
        bool anyInputActive = false;
        for (auto& input : inputs) {
            input.Update();
            if (input.active) anyInputActive = true;
        }

        if (!anyInputActive) {
            // Body Rotation
            if (IsKeyDown(keys.bodyPitchUp))   current_euler.pitch += 0.02f;
            if (IsKeyDown(keys.bodyPitchDown)) current_euler.pitch -= 0.02f;
            if (IsKeyDown(keys.bodyYawLeft))   current_euler.yaw += 0.02f;
            if (IsKeyDown(keys.bodyYawRight))  current_euler.yaw -= 0.02f;
            if (IsKeyDown(keys.bodyRollLeft))  current_euler.roll += 0.02f;
            if (IsKeyDown(keys.bodyRollRight)) current_euler.roll -= 0.02f;

            // IMU Position
            if (IsKeyDown(keys.imuPosXInc)) { imu_pos.x += 0.02f; inputs[0].SetValue(imu_pos.x); }
            if (IsKeyDown(keys.imuPosXDec)) { imu_pos.x -= 0.02f; inputs[0].SetValue(imu_pos.x); }
            if (IsKeyDown(keys.imuPosYInc)) { imu_pos.y += 0.02f; inputs[1].SetValue(imu_pos.y); }
            if (IsKeyDown(keys.imuPosYDec)) { imu_pos.y -= 0.02f; inputs[1].SetValue(imu_pos.y); }
            if (IsKeyDown(keys.imuPosZInc)) { imu_pos.z += 0.02f; inputs[2].SetValue(imu_pos.z); }
            if (IsKeyDown(keys.imuPosZDec)) { imu_pos.z -= 0.02f; inputs[2].SetValue(imu_pos.z); }

            // IMU Rotation
            if (IsKeyDown(keys.imuPitchInc)) { imu_euler.pitch += 0.02f; inputs[4].SetValue(imu_euler.pitch * RAD2DEG); }
            if (IsKeyDown(keys.imuPitchDec)) { imu_euler.pitch -= 0.02f; inputs[4].SetValue(imu_euler.pitch * RAD2DEG); }
            if (IsKeyDown(keys.imuYawLeft))  { imu_euler.yaw += 0.02f;   inputs[5].SetValue(imu_euler.yaw * RAD2DEG); }
            if (IsKeyDown(keys.imuYawRight)) { imu_euler.yaw -= 0.02f;   inputs[5].SetValue(imu_euler.yaw * RAD2DEG); }
            if (IsKeyDown(keys.imuRollLeft)) { imu_euler.roll += 0.02f;  inputs[3].SetValue(imu_euler.roll * RAD2DEG); }
            if (IsKeyDown(keys.imuRollRight)){ imu_euler.roll -= 0.02f;  inputs[3].SetValue(imu_euler.roll * RAD2DEG); }
        } else {
            imu_pos.x = inputs[0].GetValue();
            imu_pos.y = inputs[1].GetValue();
            imu_pos.z = inputs[2].GetValue();
            imu_euler.roll = inputs[3].GetValue() * DEG2RAD;
            imu_euler.pitch = inputs[4].GetValue() * DEG2RAD;
            imu_euler.yaw = inputs[5].GetValue() * DEG2RAD;
        }

        if (IsKeyPressed(keys.saveConfig)) SaveConfig(imu_pos, imu_euler);
        if (IsKeyPressed(keys.loadConfig)) {
            LoadConfig(imu_pos, imu_euler);
            inputs[0].SetValue(imu_pos.x); inputs[1].SetValue(imu_pos.y); inputs[2].SetValue(imu_pos.z);
            inputs[3].SetValue(imu_euler.roll * RAD2DEG); inputs[4].SetValue(imu_euler.pitch * RAD2DEG); inputs[5].SetValue(imu_euler.yaw * RAD2DEG);
        }
        if (IsKeyPressed(keys.resetAll)) {
            current_euler = {0,0,0}; imu_pos = {0,0,0.5f}; imu_euler = {0,0,0};
            inputs[0].SetValue(imu_pos.x); inputs[1].SetValue(imu_pos.y); inputs[2].SetValue(imu_pos.z);
            inputs[3].SetValue(imu_euler.roll * RAD2DEG); inputs[4].SetValue(imu_euler.pitch * RAD2DEG); inputs[5].SetValue(imu_euler.yaw * RAD2DEG);
        }
        quat_t body_quat = quat_from_euler(current_euler);
        quat_t sensor_to_body = quat_from_euler(imu_euler);

        BeginDrawing();
            ClearBackground(RAYWHITE);

            // --- 3D RENDERING SECTION ---
            rlViewport(0, 0, config.renderWidth, config.renderHeight); 

            BeginMode3D(camera);
                rlPushMatrix();
                    rlRotatef(90, 1, 0, 0); DrawGrid(10, 1.0f);
                rlPopMatrix();
                DrawLine3D({0,0,0}, {5,0,0}, RED); DrawLine3D({0,0,0}, {0,5,0}, GREEN); DrawLine3D({0,0,0}, {0,0,5}, BLUE);

                rlPushMatrix();
                    Quaternion ray_body_quat = ToRayQuat(body_quat);
                    Vector3 axis; float angle; QuaternionToAxisAngle(ray_body_quat, &axis, &angle);
                    rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
                    DrawCubeV({0,0,0}, {4, 2, 0.5f}, LIGHTGRAY); DrawCubeWiresV({0,0,0}, {4, 2, 0.5f}, DARKGRAY);
                    DrawLine3D({0,0,0}, {2.5f,0,0}, MAROON); DrawLine3D({0,0,0}, {0,2.5f,0}, DARKGREEN); DrawLine3D({0,0,0}, {0,0,2.5f}, DARKBLUE);

                    rlPushMatrix();
                        rlTranslatef(imu_pos.x, imu_pos.y, imu_pos.z);
                        Quaternion ray_sensor_quat = ToRayQuat(sensor_to_body);
                        QuaternionToAxisAngle(ray_sensor_quat, &axis, &angle);
                        rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
                        DrawCubeV({0,0,0}, {0.5f, 0.5f, 0.2f}, ORANGE);
                        DrawLine3D({0,0,0}, {1,0,0}, RED); DrawLine3D({0,0,0}, {0,1,0}, GREEN); DrawLine3D({0,0,0}, {0,0,1}, BLUE);
                    rlPopMatrix();
                rlPopMatrix();
            EndMode3D();

            // Reset viewport to full window for sidebar/UI elements
            rlViewport(0, 0, screenWidth, screenHeight);

            DrawRectangle(config.renderWidth, 0, config.sidebarWidth, screenHeight, Fade(SKYBLUE, 0.1f));
            DrawLine(config.renderWidth, 0, config.renderWidth, screenHeight, BLUE);

            float xPos = config.renderWidth + 20;
            float yPos = 20;
            MyDrawText("IMU CALIBRATION RIG", xPos, yPos, 24, DARKBLUE);

            yPos += 40;
            MyDrawText("BODY EULER [deg]:", xPos, yPos, 18, BLACK);
            MyDrawText(TextFormat("R:%.1f, P:%.1f, Y:%.1f", current_euler.roll*RAD2DEG, current_euler.pitch*RAD2DEG, current_euler.yaw*RAD2DEG), xPos + 10, yPos += 22, 20, DARKGRAY);

            yPos += 30;
            MyDrawText("KEYBOARD SHORTCUTS:", xPos, yPos, 18, BLACK);
            MyDrawText("- Move IMU: I/K, J/L, U/O", xPos + 10, yPos += 20, 16, DARKGRAY);
            MyDrawText("- Rotate IMU: T/G, F/H, V/B", xPos + 10, yPos += 20, 16, DARKGRAY);
            MyDrawText("- Body: W/S, A/D, Q/E", xPos + 10, yPos += 20, 16, DARKGRAY);
            MyDrawText("- [P] Save | [C] Load | [R] Reset", xPos + 10, yPos += 20, 16, MAROON);

            yPos += 30;
            MyDrawText("MANUAL IMU CONFIGURATION:", xPos, yPos, 18, BLACK);
            for (auto& input : inputs) { input.Draw(); }

            yPos = 600; 
            MyDrawText("SENSOR-TO-BODY MATRIX:", xPos, yPos, 18, MAROON);
            vec3_t c0 = quat_rotate_vec3(sensor_to_body, {1, 0, 0});
            vec3_t c1 = quat_rotate_vec3(sensor_to_body, {0, 1, 0});
            vec3_t c2 = quat_rotate_vec3(sensor_to_body, {0, 0, 1});
            MyDrawText(TextFormat("[ %.2f  %.2f  %.2f ]", c0.x, c1.x, c2.x), xPos + 10, yPos += 25, 20, BLACK);
            MyDrawText(TextFormat("[ %.2f  %.2f  %.2f ]", c0.y, c1.y, c2.y), xPos + 10, yPos += 25, 20, BLACK);
            MyDrawText(TextFormat("[ %.2f  %.2f  %.2f ]", c0.z, c1.z, c2.z), xPos + 10, yPos += 25, 20, BLACK);

        EndDrawing();
    }
    UnloadFont(mainFont);
    CloseWindow();
    return 0;
}


