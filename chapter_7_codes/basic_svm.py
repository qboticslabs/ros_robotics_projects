from sklearn import svm
import numpy as np

X = np.array([[-1, -1], [-2, -1], [1, 1], [2, 1]])
y = np.array([1, 1, 2, 2])


model = svm.SVC(kernel='linear',C=1,gamma=1)

model.fit(X,y)

print(model.predict([[-0.8,-1]]))
