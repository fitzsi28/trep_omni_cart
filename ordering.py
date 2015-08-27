import numpy as np

SS = 15 #sample size
options = np.array([1,2,3,4])
order = np.random.choice(options,1)
i1 = 0
i2 = 0
i3 = 0
i4 = 0
if order[0] == 1:
    i1+=1
elif order[0] ==2:
    i2+=1
elif order[0] ==3:
    i3+=1
else:
    i4+=1

while order.size < 4*SS:
    if i1==SS:
        index = np.where(options==1)[0][0]
        options = np.delete(options,index)
        i1+=1
    if i2==SS:
        index = np.where(options==2)[0][0]
        options = np.delete(options,index)
        i2+=1
    if i3==SS:
        index = np.where(options==3)[0][0]
        options = np.delete(options,index)
        i3+=1
    if i4==SS:
        index = np.where(options==4)[0][0]
        options = np.delete(options,index) 
        i4+=1
    temp = np.random.choice(options,1)
    if temp == 1:
        i1+=1
    elif temp ==2:
        i2+=1
    elif temp ==3:
        i3+=1
    else:
        i4+=1
        
    order = np.insert(order,-1, temp)
    
np.savetxt("test_order.csv", order, fmt="%d", delimiter=",")

    

