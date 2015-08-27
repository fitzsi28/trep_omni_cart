import numpy as np

SS = 15 #sample size
options = np.array([1,2,3,4])
order = np.random.choice(options,4,replace=False)

while order.size < 4*SS:
    temp=np.random.choice(options,4,replace=False)
    order = np.hstack([order,temp])
    
np.savetxt("test_order.csv", order, fmt="%d", delimiter=",")

    

