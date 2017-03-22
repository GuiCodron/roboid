#!/usr/bin/env python
# -*- coding: latin -*-
import random
import numpy as np
import matplotlib.pyplot as plt
import time
import os

#    liste des fonctions utiles:
#    bot(params) : constructeur de la classe, met simplement bot() si tu veux des bots aleatoires ou bot(0) pour des bots standard.
#        si tu veux creer des bot totalement perso lit le code
#    movebot(targetx,targety) : targetx et targety sont des positions absolues, cette fonction s'occupe de calculer la comande a envoyer,
#        de calculer la nouvelle position du robot et d'apprendre a partir du mouvement realise
#    disptraj(targetx,targety) : affiche la trajectoire qu'a suivi le robot ainsi qu'un cercle centre en targetx,targety
#    save() : sauvegarde les donnees de la carte de kohonen dans un fichier /!\ ca ecrase le fichier /!\
#    restore() : importe les donnees de la carte de kohonen
#    printmap() : permet d'afficher la projection de la carte de kohonen sur le plan x,y (de base les coordonnees de la carte sont x,y,pd,pg)
#    learning(epoch) : permet d'entrainer le bot sur (epoch) mouvements. Pour creer la carte de zero, epoch ~= 10^6 mais je deconseille
#
#    evite d'utiliser move et movetoward
#
#    si tu veux changer les conditions initiales, touche aux attributs posx, posy et angle juste apres avoir cree le bot.

class bot:
    def __init__(self,*parametres):
        if(len(parametres)>0):
            self.sigma=parametres[0]
        else:
            self.sigma=0.3
        if(len(parametres)>1):
            self.RrD=random.gauss(parametres[1],parametres[1]*self.sigma)
        else:
            self.RrD=random.gauss(0.145,0.145*self.sigma)
        if(len(parametres)>2):
            self.RrG=random.gauss(parametres[2],parametres[2]*self.sigma)
        else:
            self.RrG=random.gauss(0.145,0.145*self.sigma)
        if(len(parametres)>3):
            self.Rb=random.gauss(parametres[3],parametres[3]*self.sigma)
        else:
            self.Rb=random.gauss(0.3,0.3*self.sigma)
        if(len(parametres)>4):
            self.vmaxD=random.gauss(parametres[4],parametres[4]*self.sigma)
        else:
            self.vmaxD=random.gauss(572,572*self.sigma)
        if(len(parametres)>5):
            self.vmaxG=random.gauss(parametres[5],parametres[5]*self.sigma)
        else:
            self.vmaxG=random.gauss(572,572*self.sigma)
        
        self.posx=0
        self.posy=0
        self.angle=0
        
        self.alpha=0.1
        self.coeffangle=1
        self.koho_size=40
        if(len(parametres)<2):
            self.restore()
        else:
            self.koho=np.random.randn(self.koho_size,self.koho_size,5)
        self.vmax=660
        self.tracex=list()
        self.tracey=list()
        self.perf=list()
        self.perfR=list()
        
    def move(self,speedD,speedG,dt=0.001,t=0.01,reset=True,anglePerso=0,record=False):
        if(speedD>self.vmaxD):
            speedD=self.vmaxD
        if(speedD<-self.vmaxD):
            speedD=-self.vmaxD
        if(speedG>self.vmaxG):
            speedG=self.vmaxG
        if(speedG<-self.vmaxG):
            speedG=-self.vmaxG
        if(reset):
            self.posx=0
            self.posy=0
            self.angle=anglePerso
        t1=0
        while (t1<t):
            self.posx+=(self.RrD*speedD+self.RrG*speedG)*dt*np.cos(self.angle)/2
            self.posy+=(self.RrD*speedD+self.RrG*speedG)*dt*np.sin(self.angle)/2
            self.angle+=(self.RrD*speedD-self.RrG*speedG)*dt/(self.Rb*2)
            self.angle=self.angle%(2*np.pi)
            if(self.angle>np.pi):
                self.angle-=2*np.pi
            self.tracex.append(self.posx)
            self.tracey.append(self.posy)
            t1+=dt
        return(self.posx,self.posy,self.angle)
    
    
    def learning(self,epoch=500,display=False,printrate=1000):
        print("learning start!")
        start = time.time()
        xmax=0
        for i in range(epoch):
            x=0
            while(x<0.005):
                pd=random.triangular(-1,1)*self.vmax
                pg=random.triangular(-1,1)*self.vmax
                (x,y,z)=self.move(pd,pg) 
            mypoint=np.array([x,y,z,pd,pg])
            delta=mypoint-self.koho
            delta2=delta*delta
            dist=np.sum(delta2, 2)
            indice=np.argmin(dist)
            indice2=indice%self.koho_size
            indice1=indice//self.koho_size
            self.koho[indice1][indice2]+=delta[indice1][indice2]*self.alpha
            if(indice1>0):
                self.koho[indice1-1][indice2]+=delta[indice1-1][indice2]*self.alpha/2
            if(indice2>0):
                self.koho[indice1][indice2-1]+=delta[indice1][indice2-1]*self.alpha/2
            if(indice1<self.koho_size-1):
                self.koho[indice1+1][indice2]+=delta[indice1+1][indice2]*self.alpha/2
            if(indice2<self.koho_size-1):
                self.koho[indice1][indice2+1]+=delta[indice1][indice2+1]*self.alpha/2
            d=(x-self.koho[indice1][indice2][0])*(x-self.koho[indice1][indice2][0])+(y-self.koho[indice1][indice2][1])*(y-self.koho[indice1][indice2][1])
            self.perf.append(d)
            dx=(x*x+y*y)
            dr=d/dx
            if(dx<0.0000000000000000000000000000000000001):
                self.perfR.append(0)
                self.perf.append(0)
            else:
                self.perfR.append(dr)
                self.perf.append(d)
            if(x>xmax):
                xmax=x
            if(y>xmax):
                xmax=y
            if(i%printrate==0):
                print((i+1)*100/epoch,"\t%")
        
        print("xmax: ",xmax)
        if(display):
            f, axarr = plt.subplots(2, sharex=True)
            axarr[0].plot(self.perf)
            axarr[1].plot(self.perfR)
        
        npperf=np.array(self.perf)
        print(np.mean(npperf[-epoch//10:]))
        npperfR=np.array(self.perfR)
        print(np.mean(npperfR[-epoch//10:]))
        end = time.time()
        
        print("learning time: ",end - start)
        plt.show()

    def printmap(self):
        f, axarr = plt.subplots(2)
        axarr[0].axis('equal')
        axarr[0].plot(np.ravel(self.koho[:,:,0]), np.ravel(self.koho[:,:,1]), 'b,')
        axarr[1].plot(np.ravel(self.koho[:,:,3]-self.koho[:,:,4]),np.ravel(self.koho[:,:,2]),'r')
        plt.show()
        #plt.axis('equal')
        #plt.plot(np.ravel(self.koho[:,:,0]), np.ravel(self.koho[:,:,1]), 'b,')
    
    
    def save(self):
        directory = os.path.dirname(os.path.abspath(__file__))
        np.save(directory + "/mydata.npy", self.koho)
    def restore(self):
        directory = os.path.dirname(os.path.abspath(__file__))
        self.koho=np.load(directory + "/mydata.npy")
        
    def movetoward(self,targetx,targety):
        disttarget=np.sqrt(np.square(targetx)+np.square(targety))
        if(disttarget>0.005):
            if(targetx<0):
                reverse=True
                targetx=-targetx
            else:
                reverse=False
            targetangle=np.arctan2(targety,targetx)
            delta=np.array((targetx,targety))-self.koho[:,:,0:2]
            delta2=delta*delta
            dist=np.sum(delta2, 2)
            delta3=np.array([targetangle])-self.koho[:,:,2]
            dist3=delta3*delta3
            #dist3=np.sum(delta4, 2)
            dist2=dist+self.coeffangle*dist3*(dist+1)
            
            indice=np.argmin(dist2)
            indice2=indice%self.koho_size
            indice1=indice//self.koho_size
            d1=dist2[indice1,indice2]
            i2=(0,0)
            i3=(0,0)
            if(indice1>0):
                d2=dist2[indice1-1,indice2]
                i2=(indice1-1,indice2)
            else:
                d2=10000000000
            if(indice2>0):
                d3=dist2[indice1,indice2-1]
                i3=(indice1,indice2-1)
            else:
                d3=10000000000
            if(d3<d2):
                i2,i3,d2,d3=i3,i2,d3,d2
            if(indice1<self.koho_size-1 and dist2[indice1+1,indice2]<d3):
                d3=dist2[indice1+1,indice2]
                i3=(indice1+1,indice2)
            if(d3<d2):
                i2,i3,d2,d3=i3,i2,d3,d2
            if(indice2<self.koho_size-1 and dist2[indice1,indice2+1]<d3):
                d3=dist2[indice1,indice2+1]
                i3=(indice1,indice2+1)
            d1=np.sqrt(d1)
            d2=np.sqrt(d2)
            d3=np.sqrt(d3)
            if(d1!=0):
                command=(self.koho[indice1,indice2,3:5]/d1+self.koho[i2[0],i2[1],3:5]/d2+self.koho[i3[0],i3[1],3:5]/d3)/(1/d1+1/d2+1/d3)
            else:
                command=self.self.koho[indice1,indice2,3:5]
            if(reverse):
                command=-command
        else:
            command=np.array([0,0])
        return command
    
    
    
    def movebot(self,targetx,targety):
        realx=(targetx-self.posx)*np.cos(self.angle)+(targety-self.posy)*np.sin(self.angle)
        realy=(targety-self.posy)*np.cos(self.angle)-(targetx-self.posx)*np.sin(self.angle)
        command=self.movetoward(realx, realy)
        xinit=self.posx
        yinit=self.posy
        angleinit=self.angle
        self.alpha*=0.99
        self.move(command[0], command[1], reset=False,record=True)
        mypoint=np.array([self.posx-xinit,self.posy-yinit,self.angle-angleinit,command[0],command[1]])
        delta=mypoint-self.koho
        delta2=delta*delta
        dist=np.sum(delta2, 2)
        indice=np.argmin(dist)
        indice2=indice%self.koho_size
        indice1=indice//self.koho_size
        self.koho[indice1][indice2]+=delta[indice1][indice2]*self.alpha
        if(indice1>0):
            self.koho[indice1-1][indice2]+=delta[indice1-1][indice2]*self.alpha/2
        if(indice2>0):
            self.koho[indice1][indice2-1]+=delta[indice1][indice2-1]*self.alpha/2
        if(indice1<self.koho_size-1):
            self.koho[indice1+1][indice2]+=delta[indice1+1][indice2]*self.alpha/2
        if(indice2<self.koho_size-1):
            self.koho[indice1][indice2+1]+=delta[indice1][indice2+1]*self.alpha/2
        return (self.posx,self.posy,self.angle)
    
    
    
    def disptraj(self,targetx,targety,taille):
        linex=list()
        liney=list()
        for i in range(31):
            linex.append(targetx+taille*np.cos(i*2*np.pi/30))
            liney.append(targety+taille*np.sin(i*2*np.pi/30))
        plt.axis('equal')
        return plt.plot(linex,liney,'r',self.tracex,self.tracey,'b')
        
if __name__ == '__main__':
    mode=2


    if(mode==1):
        mybot=bot(0)
        #mybot.movetest(mybot.vmax,mybot.vmax)
        #mybot.restore()
        mybot.learning(1000000)
        mybot.save()
        mybot.printmap()
    if(mode==2):
        while(True):
            angle=random.uniform(0,2*np.pi)
            dist=random.uniform(90,130)
            targetx=dist*np.cos(angle)
            targety=dist*np.sin(angle)
            print("target: ",targetx,targety)
            mybot=bot(0)
            mybot.restore()
            start = time.time()
            #mybot.movetest(mybot.vmax,mybot.vmax)
            for i in range(200):
                mybot.movebot(targetx, targety)
            end = time.time()
            print("command time: ",end - start)
            p = mybot.disptraj(targetx, targety,5)
            plt.waitforbuttonpress()
            plt.close()
    #plt.waitforbuttonpress()
