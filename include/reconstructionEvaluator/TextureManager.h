#ifndef TEXTUREMANAGER_openGLUtilities_H_
#define TEXTUREMANAGER_openGLUtilities_H_

#include <string>

#include <GL/glew.h>
#include <SOIL/SOIL.h>

namespace openGLUtilities{
class TextureManager {
  public:
    TextureManager(GLenum textureTarget, const std::string& filePath);
    virtual ~TextureManager();
    bool load();
    bool load(GLint internalFormat, GLenum format, GLenum type);
    void bind(GLenum textureUnit);

  private:
    void defaultTextureParameters();
    void framebufTextureParameters();
    std::string filePath_;
    GLenum textureTarget_;
    GLuint textureObj_;

    unsigned char* image_;
};
};

#endif /* TEXTUREMANAGER_H_ */
