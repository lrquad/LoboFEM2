#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include "stb_image.h"

namespace Lobo
{
void bindShapeBuffer(unsigned int &VBO, unsigned int &EBO, std::vector<float> &buffer, std::vector<unsigned int> &indices);

void setPositionAttribute(int location, int size, int stride, int offset);

int bindTextureBuffer(const char *filename, unsigned int &texture_id);

inline void bindShapeBuffer(unsigned int &VBO, unsigned int &EBO, std::vector<float> &buffer, std::vector<unsigned int> &indices)
{

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(float), &buffer[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
}

inline void setPositionAttribute(int location, int size, int stride, int offset)
{
    glEnableVertexAttribArray(location);
    //glVertexAttribFormat()
    glVertexAttribPointer(location, size, GL_FLOAT, GL_FALSE, stride * sizeof(float), (void *)(offset * sizeof(float)));
}

inline int bindTextureBuffer(const char *filename, unsigned int &texture_id)
{
    glGenTextures(1, &texture_id);

    stbi_set_flip_vertically_on_load(true);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // load and generate the texture
    int width, height, nrChannels;

    unsigned char *data = stbi_load(filename, &width, &height, &nrChannels, 0);

    if (data && nrChannels >= 3)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else if (data && nrChannels == 1)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, width, height, 0, GL_ALPHA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }else
    {
        std::cout<<"texture info: "<<std::endl;
        std::cout << width << " " << height << " " << nrChannels << std::endl;
        std::cout << filename << std::endl;
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);
    return nrChannels;
}

inline void activeTexture(bool trigger,int textureindex, unsigned int &texture_id)
{
    if(trigger)
    {
        glActiveTexture(GL_TEXTURE0+textureindex);
        glBindTexture(GL_TEXTURE_2D,texture_id);
    }else
    {
        glActiveTexture(GL_TEXTURE0+textureindex);
        glBindTexture(GL_TEXTURE_2D,0);
    }
}

} // namespace Lobo