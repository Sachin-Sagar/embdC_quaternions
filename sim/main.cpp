#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#include "embdC_quat.h"
#include <iostream>
#include <vector>
#include <string>

// Helper to convert embdC_quat types to Raylib types
Vector3 ToRayVec(vec3_t v) { return {v.x, v.y, v.z}; }
Quaternion ToRayQuat(quat_t q) { return {q.x, q.y, q.z, q.w}; }

// Helper to draw text in 3D space (projected to 2D screen)
void DrawText3D(const char* text, Vector3 position, int fontSize, Color color, Camera3D camera) {
    Vector2 screenPos = GetWorldToScreen(position, camera);
    DrawText(text, (int)screenPos.x, (int)screenPos.y, fontSize, color);
}

int main() {
    // Initialization
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "EmbdC Quaternions - Sensor to Body Transformation");

    // Define the camera to look into our 3d world
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
    camera.up = (Vector3){ 0.0f, 0.0f, 1.0f };          // Z is UP
    camera.fovy = 45.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;             // Camera mode type

    SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second

    // Transformation State
    euler_t current_euler = {0.0f, 0.0f, 0.0f};
    quat_t sensor_to_body = quat_identity();
    
    // Example: Sensor is mounted at some orientation relative to body
    // Let's say it's tilted 30 degrees around X
    sensor_to_body = quat_from_euler({(float)(30.0 * PI / 180.0), 0, 0});

    // Main game loop
    while (!WindowShouldClose()) {
        // Update
        // Camera updates disabled per user request to prevent mouse interaction

        // Input: Rotate the "Body" using Euler angles
        if (IsKeyDown(KEY_W)) current_euler.pitch += 0.02f;
        if (IsKeyDown(KEY_S)) current_euler.pitch -= 0.02f;
        if (IsKeyDown(KEY_A)) current_euler.yaw += 0.02f;
        if (IsKeyDown(KEY_D)) current_euler.yaw -= 0.02f;
        if (IsKeyDown(KEY_Q)) current_euler.roll += 0.02f;
        if (IsKeyDown(KEY_E)) current_euler.roll -= 0.02f;

        // Reset
        if (IsKeyPressed(KEY_R)) current_euler = {0, 0, 0};

        // Compute Quaternions
        quat_t body_quat = quat_from_euler(current_euler);
        
        // Draw
        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                
                // Draw World Axes and Grid on XY Plane (Z=0)
                rlPushMatrix();
                    rlRotatef(90, 1, 0, 0); // Tilt grid to XY plane
                    DrawGrid(10, 1.0f);
                rlPopMatrix();

                DrawLine3D({0,0,0}, {5,0,0}, RED);   // X
                DrawLine3D({0,0,0}, {0,5,0}, GREEN); // Y
                DrawLine3D({0,0,0}, {0,0,5}, BLUE);  // Z (Up)

                rlPushMatrix();
                    // Apply Body Rotation
                    Quaternion ray_body_quat = ToRayQuat(body_quat);
                    Vector3 axis;
                    float angle;
                    QuaternionToAxisAngle(ray_body_quat, &axis, &angle);
                    rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
                    
                    // Draw Body (Length on X, Width on Y, Height on Z)
                    DrawCubeV({0,0,0}, {4, 2, 0.5f}, LIGHTGRAY);
                    DrawCubeWiresV({0,0,0}, {4, 2, 0.5f}, DARKGRAY);
                    
                    // Draw Body Axes
                    DrawLine3D({0,0,0}, {2.5f,0,0}, MAROON);
                    DrawLine3D({0,0,0}, {0,2.5f,0}, DARKGREEN);
                    DrawLine3D({0,0,0}, {0,0,2.5f}, DARKBLUE);

                    // Draw "Sensor" Frame (relative to body)
                    rlPushMatrix();
                        Quaternion ray_sensor_quat = ToRayQuat(sensor_to_body);
                        QuaternionToAxisAngle(ray_sensor_quat, &axis, &angle);
                        rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
                        
                        // Offset sensor slightly to the top (Z direction)
                        rlTranslatef(0, 0, 0.5f);
                        
                        DrawCubeV({0,0,0}, {0.5f, 0.5f, 0.2f}, ORANGE);
                        DrawLine3D({0,0,0}, {1,0,0}, RED);
                        DrawLine3D({0,0,0}, {0,1,0}, GREEN);
                        DrawLine3D({0,0,0}, {0,0,1}, BLUE);
                    rlPopMatrix();

                rlPopMatrix();

            EndMode3D();

            // --- 3D Labels (Drawn as 2D overlay) ---
            DrawText3D("X (World)", {5.1f, 0, 0}, 15, RED, camera);
            DrawText3D("Y (World)", {0, 5.1f, 0}, 15, GREEN, camera);
            DrawText3D("Z (World)", {0, 0, 5.1f}, 15, BLUE, camera);

            // Calculate Body Axis positions in world space for labeling
            vec3_t b_x = quat_rotate_vec3(body_quat, {2.6f, 0, 0});
            vec3_t b_y = quat_rotate_vec3(body_quat, {0, 2.6f, 0});
            vec3_t b_z = quat_rotate_vec3(body_quat, {0, 0, 2.6f});
            DrawText3D("X (Body)", ToRayVec(b_x), 15, MAROON, camera);
            DrawText3D("Y (Body)", ToRayVec(b_y), 15, DARKGREEN, camera);
            DrawText3D("Z (Body)", ToRayVec(b_z), 15, DARKBLUE, camera);

            // Label the sensor
            vec3_t s_pos = quat_rotate_vec3(body_quat, {0, 0, 0.7f});
            DrawText3D("Sensor", ToRayVec(s_pos), 15, ORANGE, camera);

            // --- Enhanced HUD ---
            DrawRectangle(5, 5, 430, 160, Fade(SKYBLUE, 0.5f));
            DrawRectangleLines(5, 5, 430, 160, BLUE);

            DrawText("EMBDC QUATERNIONS VIZ", 15, 15, 20, DARKBLUE);
            
            DrawText("ROTATION CONTROLS:", 15, 45, 12, BLACK);
            DrawText("- Pitch: W / S", 25, 60, 12, DARKGRAY);
            DrawText("- Yaw:   A / D", 25, 75, 12, DARKGRAY);
            DrawText("- Roll:  Q / E", 25, 90, 12, DARKGRAY);
            DrawText("- Reset: R", 25, 105, 12, DARKGRAY);
            
            DrawText("VIEW:", 200, 45, 12, BLACK);
            DrawText("- Static Camera", 210, 60, 12, DARKGRAY);
            DrawText("- (Mouse Disabled)", 210, 75, 12, RED);

            DrawText(TextFormat("BODY EULER [deg]: R:%.1f, P:%.1f, Y:%.1f", 
                     current_euler.roll*RAD2DEG, current_euler.pitch*RAD2DEG, current_euler.yaw*RAD2DEG), 15, 125, 14, BLACK);
            DrawText(TextFormat("BODY QUAT: W:%.2f, X:%.2f, Y:%.2f, Z:%.2f", 
                     body_quat.w, body_quat.x, body_quat.y, body_quat.z), 15, 142, 14, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
