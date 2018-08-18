import math
import time

G = 6.67428e-11

#
#class-action
#object-property
#
printout = 'false'
attempt = 0

class Vector():

      x = y = 0

      def add(self,other):

            sx , sy =self.x , self.y
            ox , oy =other.x , other.y
            fx = sx + ox
            fy = sy + oy

            result = Vector()
            result.x = fx
            result.y = fy
            
            return result

      def define(x,y):

            vector = Vector()
            vector.x = x
            vector.y = y

            return vector

      def rev(vec):

            result = Vector()
            result.x = -vec.x
            result.y = -vec.y

            return result

      def print(vec):

            print("{0:<20.2f},{1:^20.2f},{2:>20.2f}".format(vec.x,vec.y,Vector.abs(vec)))

      def ini(vec):
            
            
            vec.x = vec.y = 0

            return vec

      def abs(vec):

            return math.sqrt(vec.x**2 + vec.y**2)

      def mul(vector,constant):

            result = Vector()
            result.x = constant * vector.x
            result.y = constant * vector.y

            return result

      def ref(vector):

            constant = 1/Vector.abs(vector)
            refvector = Vector.mul(vector,constant)
            
            return refvector

      def sub(start,end):
            #
            ###result pointing from self to other
            ###other = end, self = start
            #
            Vector_sub = Vector()
            Vector_sub = Vector.add(Vector.rev(start),end)
            
            return Vector_sub

      
class Body():

      pos = Vector()
      Vector.ini(pos)
      
      vel = Vector()
      Vector.ini(vel)

      force = Vector()
      Vector.ini(force)

      acc = Vector()
      Vector.ini(acc)

      name = None
      types = None
      mass = None
      radius = None
      thrust = None
      program = None
      phase = 'ini'

      refpos = Vector.define(0,0)
      refvel = Vector.define(0,0)
      refmass = None
      refphase = 'ini'
      refstep = None
      stepcounter = 0 #countdown

      currentstep = None

      
      def attraction(self,other):
            #
            ##attraction of others acting on self
            #
            displacement = Vector.add(Vector.rev(self.pos),other.pos)
            d = Vector.abs(displacement)
            forceMag = G*self.mass*other.mass/(d**2)
            forcevec = Vector.mul(Vector.ref(displacement),forceMag)
            
            return forcevec

      def print(body):

            Vector.print(body.pos)
            Vector.print(body.vel)
            print(body.mass)            

                  
def Guidance(bodies,step):

      global printout
      global attempt
      ini = 0
      
      for item in bodies:

            if item.types == 'propulsive':
                  spacecraft = item
            else:primary = item
           
      velocity_relative = Vector()
      velocity_relative = Vector.sub(primary.vel,spacecraft.vel)
      d = Vector.abs(Vector.sub(primary.pos,spacecraft.pos))
      
      def deorbit(primary,spacecraft,step):
            
            a = (d + primary.radius)/2
            
            target_velocity = math.sqrt((2/d - 1/a)*G*primary.mass)
            
            if Vector.abs(velocity_relative) >= target_velocity :

                  thrust_force = Vector()
                  Vector.ini(thrust_force)
                  thrust_force = Vector.mul(Vector.rev(Vector.ref(velocity_relative)),spacecraft.thrust)
                  
                  return thrust_force

            else:
                  spacecraft.phase = 'freefall'
                  thrust_force = Vector.define(0,0)
                  
                  spacecraft.refstep = step
                  spacecraft.stepcounter = 0

                  spacecraft.refmass = spacecraft.mass
                  spacecraft.refpos = spacecraft.pos
                  spacecraft.refvel = spacecraft.vel
                  spacecraft.refphase = spacecraft.phase

                  primary.refpos = primary.pos
                  primary.refvel = primary.vel
                  
                  
                  return thrust_force

      def freefall(primary,spacecraft,step):

            thrust_force = Vector()
            Vector.ini(thrust_force)

            if step <= spacecraft.stepcounter + spacecraft.refstep:

                  thrust_force = Vector.define(0,0)

            elif Vector.abs(velocity_relative) >  10:
                  
                  thrust_force = Vector.mul(Vector.rev(Vector.ref(velocity_relative)),spacecraft.thrust)

            else:

                  spacecraft.phase = 'terminal'

                  thrust_force = Vector.define(0,0)
                  
            return thrust_force

      def terminal(spacecraft,primary):

            if d < primary.radius + 100 and d > primary.radius + 1:

                  acc_target = Vector.abs(velocity_relative) ** 2 / ((d-primary.radius)*2) + 1.625 #lunar gravity
                  thrust_force = Vector.mul(Vector.rev(Vector.ref(velocity_relative)),spacecraft.mass * acc_target)
                  
            else:
                  thrust_force = Vector.define(0,0)
                  spacecraft.phase = 'evaluation'

            return thrust_force


      if spacecraft.phase == 'ini':

            thrust_force = deorbit(primary,spacecraft,step)
            return thrust_force
            

      if spacecraft.phase == 'freefall':

            thrust_force = freefall(primary,spacecraft,step)
            
            return thrust_force

      if spacecraft.phase == 'terminal':

            thrust_force = terminal(spacecraft,primary)
            return thrust_force
      
      if spacecraft.phase == 'evaluation':

            attempt += 1
            
            if d>=primary.radius + 1 :
                  #premature
                  #reverting

                  spacecraft.mass = spacecraft.refmass
                  spacecraft.pos = spacecraft.refpos
                  spacecraft.vel = spacecraft.refvel
                  spacecraft.phase = spacecraft.refphase
                  spacecraft.currentstep = spacecraft.refstep

                  primary.vel = primary.refvel
                  primary.pos = primary.refpos

                  thrust_force = Vector.define(0,0)

                  spacecraft.stepcounter += 1

            elif d < primary.radius - 1 :
                  print("overshoot")
                  update_info(bodies,step)
                  print(spacecraft.mass)
                  exit()
            else:
                  print("success")
                  update_info(bodies,step)
                  exit()

            return thrust_force

      
                  
def update_info(bodies,tick):

      global attempt
      

      print("\n\n","{0:^7}".format(tick))
      print("no.",attempt)
      
      for body in bodies:

            if body.name == 'moon':
                  continue
            print(body.name)
            Vector.print(body.pos)
            Vector.print(body.vel)
            print(body.mass)

      #time.sleep(0.1)

      
      
def EulerInt(bodies):

      global printout

      timestep = 0.5
      tick = 0
      step = 0
      
      while True:
            
            for body in bodies:

                  Vector.ini(body.force)

            for body in bodies:

                  if body.types == 'propulsive':

                        body.currentstep = step
                        thrust_force = Guidance(bodies,step)
                        step = body.currentstep
                        body.mass -= (Vector.abs(thrust_force)/(body.isp*9.8)) * timestep
                        body.force = Vector.add(thrust_force,body.force)
                        pass
                  
            for body in bodies:
                  
                  for other in bodies:
                        
                        if body is other:
                              continue
                        
                        body.force = Vector.add(Body.attraction(body,other),body.force)

                  
            for body in bodies:


                  body.acc = Vector.mul(body.force,1/body.mass)
                  body.vel = Vector.add(body.vel,Vector.mul(body.acc,timestep))
                  body.pos = Vector.add(body.pos,Vector.mul(body.vel,timestep))

            tick += timestep
            step += 1

            if step % 2 ==0 and printout == 'true':
                  update_info(bodies,tick)
                 
                        
                        
def main():


      Moon = Body()
      Moon.name = 'moon'
      Moon.pos = Vector.define(0,0)
      Moon.vel = Vector.define(0,0)
      Moon.mass = 7.34767309 * 10**22
      Moon.types = 'free'
      Moon.radius = 1737 * 10**3

      testobj = Body()
      testobj.name = 'testobj'
      testobj.pos = Vector.define(0,Moon.radius+100000)
      testobj.vel = Vector.define(1633,0)
      testobj.mass = 25000
      testobj.types = 'propulsive'
      testobj.radius = 1
      testobj.thrust = 75000
      testobj.isp = 320 
      testobj.program = 1
      Vector.print(testobj.vel)
      
      EulerInt([Moon,testobj])
      Vector.print(Moon.force)
      Vector.print(testobj.force)


if __name__ == '__main__':
    main()      
