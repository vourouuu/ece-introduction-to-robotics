// * File:          SupervisorController.c
// * Date:          09/11/2024
// * Description:   Controls Cylinder's and Box's orientation
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
                   
  /********************************************************** "RectangleArena" node (World) *********************************************************/
  Matrix3x3 temp_w = multiply_matrices(Rx(0.0), Ry(0.0));
  Matrix3x3 R_w = multiply_matrices(temp_w, Rz(0.0));
  double p_w[3] = {0.0, 0.0, 0.0};
  Matrix4x4 T_w = T(R_w, p_w);
  /********************************************************** "RectangleArena" node (World) *********************************************************/
  
  /***************************************************************** "red-box" node *****************************************************************/
  Matrix3x3 temp1_wb = multiply_matrices(Rx(0.0), Ry(0.0));
  Matrix3x3 temp2_wb  = multiply_matrices(temp1_wb, Rz(0.0));
  Matrix3x3 R_wb = multiply_matrices(temp2_wb, R_w);
  double p_wb[3] = {0.5, 0.5, 0.1};
  Matrix4x4 T_wb = T(R_wb, p_wb);
  /***************************************************************** "red-box" node *****************************************************************/
    
  /************************************************************* "orange-cylinder" node *************************************************************/
  Matrix3x3 temp1_wc = multiply_matrices(Rx(0.0), Ry(-90.0));
  Matrix3x3 temp2_wc  = multiply_matrices(temp1_wc, Rz(0.0));
  Matrix3x3 R_wc = multiply_matrices(temp2_wc, R_w);
  double p_wc[3] = {0.695, 0.095, 0.025};
  Matrix4x4 T_wc = T(R_wc, p_wc);
  /************************************************************* "orange-cylinder" node *************************************************************/
 
  
  
  /*********************************************** Move the "red-box" node to the correct position **************************************************/
  WbNodeRef BoxNode = wb_supervisor_node_get_from_def("red-box"); // represents the solid "red-box" node from the world
  WbFieldRef translation_field_of_box = wb_supervisor_node_get_field(BoxNode, "translation");
  
  double p_wb_pos[3];
  memcpy(p_wb_pos,p_wb,3*sizeof(double));
  wb_supervisor_field_set_sf_vec3f(translation_field_of_box, p_wb_pos);
  /*********************************************** Move the "red-box" node to the correct position **************************************************/
    
  /******************************************* Move the "orange-cylinder" node to the correct position **********************************************/
  WbNodeRef CylinderNode = wb_supervisor_node_get_from_def("orange-cylinder"); // represents the solid "orange-cylinder" node from the world
  WbFieldRef translation_field_of_cyl = wb_supervisor_node_get_field(CylinderNode, "translation");

  double p_wc_pos[3];
  memcpy(p_wc_pos,p_wc,3*sizeof(double));
  wb_supervisor_field_set_sf_vec3f(translation_field_of_cyl, p_wc_pos);
  /******************************************* Move the "orange-cylinder" node to the correct position **********************************************/
  
  
  
  /********************************************** Rotate the "red-box" node to the correct orientation **********************************************/
  WbFieldRef rotation_field_of_box = wb_supervisor_node_get_field(BoxNode, "rotation");
  const double R_wb_rot[4] = {1, 0, 0, 0}; // axis: x (or y or z), rotation: 0 rad [0 degrees]
  wb_supervisor_field_set_sf_rotation(rotation_field_of_box, R_wb_rot);
  /********************************************** Rotate the "red-box" node to the correct orientation **********************************************/
    
  /****************************************** Rotate the "orange-cylinder" node to the correct orientstion ******************************************/
  WbFieldRef rotation_field_of_cyl = wb_supervisor_node_get_field(CylinderNode, "rotation");
  const double R_wc_rot[4] = {0, 1, 0, -1.5708}; // axis: y, rotation: -1.5708 rad [-90 degrees]
  wb_supervisor_field_set_sf_rotation(rotation_field_of_cyl, R_wc_rot);
  /****************************************** Rotate the "orange-cylinder" node to the correct orientation ******************************************/

  wb_robot_step(TIME_STEP * 20);

  /************************** Cylinder's Transformation Matrix when it is above the hole oriented in order to fit inside it *************************/
  Matrix3x3 temp1_wc_new = multiply_matrices(Rx(0.0), Ry(90.0));
  Matrix3x3 temp2_wc_new = multiply_matrices(temp1_wc_new, Rz(0.0));
  Matrix3x3 R_wc_new = multiply_matrices(R_wc, temp2_wc_new);
  double p_wc_new[3] = {0.5, 0.5, 0.325};
  Matrix4x4 T_wc_new = T(R_wc_new, p_wc_new);
  
  translation_field_of_cyl = wb_supervisor_node_get_field(CylinderNode, "translation");
  double p_wc_pos_new[3];
  memcpy(p_wc_pos_new,p_wc_new,3*sizeof(double));
  wb_supervisor_field_set_sf_vec3f(translation_field_of_cyl, p_wc_pos_new);
  
  const double R_wc_rot_new[4] = {1, 0, 0, 0}; // axis: x, rotation: 0 rad [0 degrees]
  wb_supervisor_field_set_sf_rotation(rotation_field_of_cyl, R_wc_rot_new);
  /************************** Cylinder's Transformation Matrix when it is above the hole oriented in order to fit inside it *************************/

  /* Printing matrices on the console */
  printf("\n\n> World's Transformation Matrix (Tw):\n");
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      printf("%7.3lf ", T_w.matrix[i][j]);
    }
    printf("\n");
  }
  
  printf("\n\n> Box's Transformation Matrix (Twb):\n");
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      printf("%7.3lf ", T_wb.matrix[i][j]);
    }
    printf("\n");
  }
  
  printf("\n\n> Cylinder's Transformation Matrix (Twc):\n");
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      printf("%7.3lf ", T_wc.matrix[i][j]);
    }
    printf("\n");
  }
  
  printf("\n\n> Cylinder's Transformation Matrix when it is above the hole (T'wc):\n");
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      printf("%7.3lf ", T_wc_new.matrix[i][j]);
    }
    printf("\n");
  }
  
  wb_robot_cleanup();
  
  return 0;
}