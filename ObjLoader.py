import pygame
import os
from OpenGL.GL import *

def MTL(filename, mtl_texture_id, texture_file_override):
    contents = {}
    mtl = None
    for line in open(filename, "r"):
        if line.startswith('#'): continue
        values = line.split()
        if not values: continue
        if values[0] == 'newmtl':
            mtl = contents[values[1]] = {}
        elif mtl is None:
            raise ValueError, "mtl file doesn't start with newmtl stmt"
        elif values[0] == 'map_Kd':
            # load the texture referred to by this declaration
            print texture_file_override
            if texture_file_override is "OBJ":
            	mtl[values[0]] = values[1]
            	surf_texture_file = os.path.dirname(os.path.abspath(__file__)) + "/" + mtl['map_Kd']
            else:
            	surf_texture_file = os.path.dirname(os.path.abspath(__file__)) + "/" + texture_file_override
            surf = pygame.image.load(surf_texture_file)
            image = pygame.image.tostring(surf, 'RGBA', 1)
            ix, iy = surf.get_rect().size
            texid = mtl['texture_Kd'] = mtl_texture_id
            glBindTexture(GL_TEXTURE_2D, mtl_texture_id)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                GL_LINEAR)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA,
                GL_UNSIGNED_BYTE, image)
        else:
            mtl[values[0]] = map(float, values[1:])
    return contents
 
class OBJ:
    def __init__(self, filename, texture_file_override, swapyz=False, ):
        """Loads a Wavefront OBJ file. """
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []
        texture_list = glGenTextures(2)
        
        material = None
        for line in open(filename, "r"):
            if line.startswith('#'): continue
            values = line.split()
            if not values: continue
            if values[0] == 'v':
                v = map(float, values[1:4])
                if swapyz:
                    v = v[0], v[2], v[1]
                self.vertices.append(v)
            elif values[0] == 'vn':
                v = map(float, values[1:4])
                if swapyz:
                    v = v[0], v[2], v[1]
                self.normals.append(v)
            elif values[0] == 'vt':
                self.texcoords.append(map(float, values[1:3]))
            elif values[0] in ('usemtl', 'usemat'):
                material = values[1]
            elif values[0] == 'mtllib':
                mtl_path = os.path.dirname(os.path.abspath(__file__)) + "/" + values[1]
                self.mtl = MTL(mtl_path, texture_list[1], texture_file_override)
            elif values[0] == 'f':
                face = []
                texcoords = []
                norms = []
                for v in values[1:]:
                    w = v.split('/')
                    face.append(int(w[0]))
                    if len(w) >= 2 and len(w[1]) > 0:
                        texcoords.append(int(w[1]))
                    else:
                        texcoords.append(0)
                    if len(w) >= 3 and len(w[2]) > 0:
                        norms.append(int(w[2]))
                    else:
                        norms.append(0)
                self.faces.append((face, norms, texcoords, material))
        
        surf_texture_file = os.path.dirname(os.path.abspath(__file__)) + "/" + "pica.jpg"
        surf = pygame.image.load(surf_texture_file)
        image = pygame.image.tostring(surf, 'RGBA', 1)
        ix, iy = surf.get_rect().size
        
        
        print texture_list
        glBindTexture(GL_TEXTURE_2D, texture_list[0])
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
            GL_LINEAR)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA,
                     GL_UNSIGNED_BYTE, image)
        
        self.gl_list = glGenLists(1)
        glEnable(GL_TEXTURE_2D)
        glNewList(self.gl_list, GL_COMPILE)
        #glActiveTexture(GL_TEXTURE0)
        glBindTexture(GL_TEXTURE_2D, texture_list[0])
        glBegin(GL_TRIANGLES)
        glTexCoord2f(0, 0)
        glVertex3f( -1,  0, -1 )
        glTexCoord2f(1, 0)
        glVertex3f(  1,  0, -1 )
        glTexCoord2f(0, 1)
        glVertex3f( -1,  1, -1 )
         
        # second triangle, top right half
        glTexCoord2f(1, 0)
        glVertex3f(  1,  0, -1 )
        glTexCoord2f(0, 1)
        glVertex3f( -1,  1, -1 )
        glTexCoord2f(1, 1)
        glVertex3f(  1,  1, -1 )
        glEnd()
        
        glFrontFace(GL_CCW)
        for face in self.faces:
            vertices, normals, texture_coords, material = face
  
            mtl = self.mtl[material]
            if 'texture_Kd' in mtl:
                # use diffuse texmap
                #glActiveTexture(GL_TEXTURE0)
                glBindTexture(GL_TEXTURE_2D, mtl['texture_Kd'])
            else:
                #just use diffuse colour
                glColor(*mtl['Kd'])
  
            glBegin(GL_POLYGON)
            
            for i in range(len(vertices)):
                if normals[i] > 0:
                    glNormal3fv(self.normals[normals[i] - 1])
                if texture_coords[i] > 0:
                    glTexCoord2fv(self.texcoords[texture_coords[i] - 1])
                glVertex3fv(self.vertices[vertices[i] - 1])
            glEnd()
       
        
        glEndList()
