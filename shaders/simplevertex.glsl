
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in vec3 aColor;
#define NR_POINT_LIGHTS 6

out vec3 ourColor;
out vec2 TexCoords;
out vec3 ourNormal;
out vec3 FragPos;
out vec3 eyePos;
out vec4 FragPosLightSpace[NR_POINT_LIGHTS];

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform mat4 lightSpaceMatrix[NR_POINT_LIGHTS];

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0f);
    
    FragPos = vec3(model * vec4(aPos, 1.0));
    eyePos = vec3(gl_Position);
    
    TexCoords = aTexCoord;
    ourColor = aColor;
    mat3 normalMatrix = transpose(inverse(mat3(model)));
    ourNormal = normalMatrix * aNormal; 

    for (int i=0;i<NR_POINT_LIGHTS;i++)
    FragPosLightSpace[i] = lightSpaceMatrix[i] * vec4(FragPos, 1.0);
    //TexCoord = aTexCoord;

}