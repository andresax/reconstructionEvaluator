#version 420

in vec3 position;
out vec3 positionPV;
out vec3 positionPextrV;
out vec4 shadowCoordV;  

uniform mat4 MVP;
uniform mat4 E;
uniform vec3 center;
uniform sampler2DShadow shadowMap;

void main(){

  mat4 biasMatrix = mat4(
    0.5, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.5, 0.5, 0.5, 1.0
    );
  shadowCoordV =  MVP * vec4(position.xyz, 1.0);
  shadowCoordV = biasMatrix * shadowCoordV;
  positionPV = position.xyz;
  vec4 positionPextrVtmp =  vec4(position.xyz, 1.0)*E;
  // positionPextrVtmp.x = positionPextrVtmp.x/positionPextrVtmp.w;
  // positionPextrVtmp.y = positionPextrVtmp.y/positionPextrVtmp.w;
  // positionPextrVtmp.z = positionPextrVtmp.z/positionPextrVtmp.w;

  positionPextrV = positionPextrVtmp.xyz;
  gl_Position =  MVP * vec4(position.xyz,1.0);
}

