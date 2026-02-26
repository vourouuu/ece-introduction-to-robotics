// * File:          PandaController.c
// * Date:          09/11/2024
// * Description:   Controls Panda's orientation
// * Author:        Vrachoriti Alexandra

#include <stdio.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 32

WbNodeRef wb_supervisor_node_get_from_def(const char *def);                      // returns a handle to a node in the world from its DEF name
WbFieldRef wb_supervisor_node_get_field(WbNodeRef node, const char *field_name); // retrieves a handler to a node field
void wb_supervisor_field_set_sf_vec3f(WbFieldRef field, const double values[3]);
void wb_supervisor_field_set_sf_rotation(WbFieldRef field, const double values[4]);

typedef struct {
  double matrix[3][3];
} Matrix3x3;

typedef struct {
  double matrix[4][4];
} Matrix4x4;

Matrix3x3 Rx(double theta) {
  Matrix3x3 matrix = {{
    {1.0,               0.0,                0.0},
    {0.0, round(cos(theta)), round(-sin(theta))},
    {0.0, round(sin(theta)),  round(cos(theta))}
  }};
  
  return matrix;
}

Matrix3x3 Ry(double theta) {
  Matrix3x3 matrix = {{
    { round(cos(theta)), 0.0, round(sin(theta))},
    {               0.0, 1.0,               0.0},
    {round(-sin(theta)), 0.0, round(cos(theta))}
  }};
  
  return matrix;
}

Matrix3x3 Rz(double theta) {
  Matrix3x3 matrix = {{
    {round(cos(theta)), round(-sin(theta)), 0.0},
    {round(sin(theta)),  round(cos(theta)), 0.0},
    {              0.0,                0.0, 1.0}
  }};
  
  return matrix;
}

Matrix3x3 multiply_matrices(Matrix3x3 matrix1, Matrix3x3 matrix2) {
  Matrix3x3 result = {0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        result.matrix[i][j] += matrix1.matrix[i][k] * matrix2.matrix[k][j];
      }
    }
  }
  return result;
}

Matrix4x4 T(Matrix3x3 R, double p[3]) {
  Matrix4x4 transf_matrix = {{
    {R.matrix[0][0], R.matrix[0][1], R.matrix[0][2], p[0]},
    {R.matrix[1][0], R.matrix[1][1], R.matrix[1][2], p[1]},
    {R.matrix[2][0], R.matrix[2][1], R.matrix[2][2], p[2]},
    {           0.0,            0.0,            0.0,  1.0}
  }};
  
  return transf_matrix;
}

int main(int argc, char **argv) {
  wb_robot_init(); // initializes the Webots controller library and enables
                   // the communication with the Webots simulator

  /********************************************************* "RectangleArena" node (World) ********************************************************/
  Matrix3x3 temp_w = multiply_matrices(Rx(0.0), Ry(0.0));
  Matrix3x3 R_w = multiply_matrices(temp_w, Rz(0.0));
  /********************************************************* "RectangleArena" node (World) ********************************************************/
  
  /****************************************************************** "ARM" node ******************************************************************/
  Matrix3x3 temp1_wbase = multiply_matrices(Rx(0.0), Ry(0.0));
  Matrix3x3 temp2_wbase = multiply_matrices(temp1_wbase, Rz(-90.0));
  Matrix3x3 R_wbase = multiply_matrices(temp2_wbase, R_w);
  double p_wbase[3] = {0.5, 0.735, 0.0};
  Matrix4x4 T_wbase = T(R_wbase, p_wbase);
  /****************************************************************** "ARM" node ******************************************************************/
  
  
  
  /************************************************ Move the "ARM" node to the correct position ***************************************************/
  WbNodeRef ArmNode = wb_supervisor_node_get_from_def("ARM"); // represents the robot "ARM" node from the world
  WbFieldRef translation_field_of_Panda = wb_supervisor_node_get_field(ArmNode, "translation");

  double p_wbase_pos[3];
  memcpy(p_wbase_pos,p_wbase,3*sizeof(double));
  wb_supervisor_field_set_sf_vec3f(translation_field_of_Panda, p_wbase_pos);
  /************************************************ Move the "ARM" node to the correct position ***************************************************/
  
  
  
  /*********************************************** Rotate the "ARM" node to the correct orientation ***********************************************/
  WbFieldRef rotation_field_of_arm = wb_supervisor_node_get_field(ArmNode, "rotation");
  const double R_wbase_rot[4] = {0, 0, 1, -1.5708}; // axis: z, rotation: -1.5708 rad [-90 degrees]
  wb_supervisor_field_set_sf_rotation(rotation_field_of_arm, R_wbase_rot);
  /*********************************************** Rotate the "ARM" node to the correct orientation ***********************************************/
  

  /* Printing matrix on the console */
  printf("\n\n> Base-frame's Transformation Matrix (Twbase):\n");
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      printf("%7.3lf ", T_wbase.matrix[i][j]);
    }
    printf("\n");
  }
  
  wb_robot_cleanup();
  
  return 0;
}