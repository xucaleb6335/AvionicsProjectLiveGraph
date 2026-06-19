package com.kagenou.avionics.scene;
import static org.lwjgl.opengl.GL11.*;
public class Cube {
    //12 triangles, 2 for each face
    private float[] vertices = {
            // Front face
            -0.5f, -0.25f,  0.5f,
            0.5f, -0.25f,  0.5f,
            0.5f,  0.25f,  0.5f,
            -0.5f,  0.25f,  0.5f,
            // Back face
            -0.5f, -0.25f, -0.5f,
            -0.5f,  0.25f, -0.5f,
            0.5f,  0.25f, -0.5f,
            0.5f, -0.25f, -0.5f,
            // Left face
            -0.5f, -0.25f, -0.5f,
            -0.5f, -0.25f,  0.5f,
            -0.5f,  0.25f,  0.5f,
            -0.5f,  0.25f, -0.5f,
            // Right face
            0.5f, -0.25f, -0.5f,
            0.5f,  0.25f, -0.5f,
            0.5f,  0.25f,  0.5f,
            0.5f, -0.25f,  0.5f,
            // Top face
            -0.5f,  0.25f, -0.5f,
            -0.5f,  0.25f,  0.5f,
            0.5f,  0.25f,  0.5f,
            0.5f,  0.25f, -0.5f,
            // Bottom face
            -0.5f, -0.25f, -0.5f,
            0.5f, -0.25f, -0.5f,
            0.5f, -0.25f,  0.5f,
            -0.5f, -0.25f,  0.5f,
    };

    private int[] indices = {
            0, 1, 2, 2, 3, 0,  // front face
            4, 5, 6, 6, 7, 4,  // back face
            8, 9, 10, 10, 11, 8, // left face
            12, 13, 14, 14, 15, 12, // right face
            16, 17, 18, 18, 19, 16, // top face
            20, 21, 22, 22, 23, 20  // bottom face
    };

    public void draw() {
        glBegin(GL_TRIANGLES);

        for(int i =0; i <indices.length;i++){
            int vertexIdx = indices[i]*3;

            //Use switch case for each of 6 sides
            switch(i/6) {
                case 0: glColor3f(1,0,0); break;
                case 1: glColor3f(0,1,0); break;
                case 2: glColor3f(0,0,1); break;
                case 3: glColor3f(1,1,0); break;
                case 4: glColor3f(0,1,1); break;
                case 5: glColor3f(1,0,1); break;
            }
            glVertex3f(vertices[vertexIdx], vertices[vertexIdx+1],vertices[vertexIdx+2]);
        }

        glEnd(); //Important line so GL actually draws
    }
}