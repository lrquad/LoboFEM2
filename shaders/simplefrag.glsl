#version 330 core
#define NR_POINT_LIGHTS 6

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    sampler2D diffuse_tex;
    sampler2D emissive_tex;
    sampler2D normal_tex;
    sampler2D bump_tex;

    float shininess;
}; 

struct Lights {
    vec3 position;
    vec3 direction;
    
    float constant;
    float linear;
    float quadratic;
	
    vec3 lightColor;
    bool trigger;
    bool cast_shadow;
    int light_type; //0 point 1 directional

};
  
uniform Material material;

out vec4 FragColor;
  
in vec3 ourColor;
in vec2 TexCoords;
in vec3 ourNormal;
in vec3 FragPos;
in vec3 eyePos;
in vec4 FragPosLightSpace[NR_POINT_LIGHTS];
uniform sampler2D shadowMap[NR_POINT_LIGHTS];


//uniform vec3 lightPos; 
//uniform vec3 lightColor;
uniform vec3 viewPos;  

uniform bool useDiffuseTex = false;
uniform bool useNormalTex = false;
uniform bool useBumpTex = false;
uniform bool useFlatNormal = true;
uniform bool vertex_color_mode = false;
uniform bool use_blinn = true;


uniform Lights lights[NR_POINT_LIGHTS];

vec3 CalcDirLight(Lights light, vec3 normal, vec3 viewDir,float shadow);
vec3 CalcPointLight(Lights light, vec3 normal, vec3 fragPos, vec3 viewDir,float shadow);
float ShadowCalculation(Lights light,vec4 fragPosLightSpace,sampler2D shadowMap);

void main()
{
    // properties
    vec3 norm = normalize(ourNormal);
    
    if(useFlatNormal)
    norm = normalize(cross(dFdx(FragPos), dFdy(FragPos)));

    vec3 tmp_norm = norm;

    if(useNormalTex)
    {
        vec3 N = norm;
        vec3 q0 = dFdx(eyePos.xyz);
        vec3 q1 = dFdy(eyePos.xyz);
        vec2 st0 = dFdx(TexCoords.st);
        vec2 st1 = dFdy(TexCoords.st);
        vec3 S = normalize( q0 * st1.t - q1 * st0.t);
        vec3 T = normalize(-q0 * st1.s + q1 * st0.s);
        
        vec3 NfromST = cross( S, T );
        if( dot( NfromST, N ) < 0.0 ) {
            S *= -1.0;
            T *= -1.0;
        }
        mat3 TBN = mat3(S, T, norm);
        norm = texture(material.normal_tex, TexCoords).rgb;
        norm = (norm * 2.0 - 1.0);  // this normal is in tangent space
        norm = normalize((TBN)*norm);
    }

    vec3 viewDir = normalize(viewPos - FragPos);

    vec3 result = vec3(0.0);
    for(int i = 0; i < NR_POINT_LIGHTS; i++)
    {
        if(lights[i].trigger==true)
        {
            float shadow = ShadowCalculation(lights[i],FragPosLightSpace[i],shadowMap[i]);  
            if(!lights[i].cast_shadow)
            {
                shadow = 0.0;
            }
            if (lights[i].light_type==0)
                result += CalcPointLight(lights[i], norm, FragPos, viewDir,shadow);  
            if (lights[i].light_type==1)
                result += CalcDirLight(lights[i], norm, FragPos,shadow);  
        }
    }  
    
    FragColor = vec4(result, 1.0);
    float gamma = 2.2;
    FragColor.rgb = pow(FragColor.rgb, vec3(1.0/gamma));
}

vec3 CalcDirLight(Lights light, vec3 normal, vec3 viewDir,float shadow)
{
    vec3 lightDir = normalize(-light.direction);
    // diffuse shading
    float diff = max(dot(normal, lightDir), 0.0);
    // specular shading
    float spec =0.0;
    if(use_blinn)
    {
        vec3 halfwayDir = normalize(lightDir + viewDir);  
        spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    }else
    {
        vec3 reflectDir = reflect(-lightDir, normal);
        spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    }
    // combine results
    vec3 ambient = light.lightColor * material.ambient;
    vec3 diffuse = light.lightColor * diff * material.diffuse;
    vec3 specular = light.lightColor * spec * material.specular;

    if (useDiffuseTex == true)
    {
        diffuse = light.lightColor * diff * vec3(texture(material.diffuse_tex, TexCoords)); 
        vec3 emission = texture(material.emissive_tex, TexCoords).rgb;
        diffuse+=emission;
    }

    if(vertex_color_mode==true)
    {
        diffuse = light.lightColor * diff *ourColor;
    }

    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular));    
    return lighting;
}

vec3 CalcPointLight(Lights light, vec3 normal, vec3 fragPos, vec3 viewDir,float shadow)
{
    vec3 lightDir = normalize(light.position - fragPos);
    // diffuse shading
    float diff = max(dot(normal, lightDir), 0.0);
    // specular shading
    
    float spec =0.0;

    if(use_blinn)
    {
        vec3 halfwayDir = normalize(lightDir + viewDir);  
        spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    }else
    {
        vec3 reflectDir = reflect(-lightDir, normal);
        spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    }

    // attenuation
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    
    // combine results
    vec3 ambient = light.lightColor * material.ambient;
    vec3 diffuse = light.lightColor * diff * material.diffuse;
    vec3 specular = light.lightColor * spec * material.specular;

    if (useDiffuseTex == true)
    {
        diffuse = light.lightColor * diff * vec3(texture(material.diffuse_tex, TexCoords)); 
        vec3 emission = texture(material.emissive_tex, TexCoords).rgb;
        diffuse+=emission;
    }

    if(vertex_color_mode==true)
    {
        diffuse = light.lightColor * diff *ourColor;
    }

    ambient *= attenuation;
    diffuse *= attenuation;
    specular *= attenuation;
    //return material.ambient;
    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular));    
    return lighting;
}

float ShadowCalculation(Lights light,vec4 fragPosLightSpace,sampler2D shadowMap)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(shadowMap, projCoords.xy).r; 
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;

    // calculate bias (based on depth map resolution and slope)
    vec3 normal = normalize(ourNormal);
    vec3 lightDir = normalize(light.position - FragPos);
    float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.004);
    bias = 0.0;
    // check whether current frag pos is in shadow
    // float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0;
    // PCF
    float shadow = 0.0;
    vec2 texelSize = 10.0 / textureSize(shadowMap, 0);
    for(int x = -3; x <= 3; ++x)
    {
        for(int y = -3; y <= 3; ++y)
        {
            vec2 offset = vec2(x, y);
            float pcfDepth = texture(shadowMap, projCoords.xy + offset * texelSize).r; 
            float contri_shadow = currentDepth - bias > pcfDepth  ? 1.0 : 0.0; 
            shadow += contri_shadow;        
        }    
    }

    //float pcfDepth = texture(shadowMap, projCoords.xy).r; 
    //float contri_shadow = currentDepth - bias > pcfDepth  ? 1.0 : 0.0;

    shadow /= 49.0;
    //shadow*=0.95;
    
    // keep the shadow at 0.0 when outside the far_plane region of the light's frustum.
    if(projCoords.z > 1.0)
        shadow = 0.0;
        
    return shadow;
}