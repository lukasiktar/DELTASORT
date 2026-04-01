import pandas as pd
import numpy as np
import joblib
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import LeaveOneOut, cross_val_score

df = pd.read_csv("model_train_data02.csv", header = None, names=["label","S0","S1","S2","S3","diff_01","diff_02","diff_03","diff_12","diff_13","diff_23"])

X = df.drop("label", axis=1).values.astype(np.float32) #Removes the label column and converts pd to numpy
y =(df["label"]=="HDPE").astype(int).values #Transform labels to 0 for PET and 1 for HDPE

#Scale the inputs aorund the mean value
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

#Support vector classifier
clf = SVC(kernel='rbf', C=1.0, probability=True)
#RBF kernel - Radial basis functon - curved boundary between data - if data is not linearly separable

#Evaluate
scores = cross_val_score(clf, X_scaled, y, cv=LeaveOneOut())
print(f"LOO Accuracy: {scores.mean()*100:.1f}")

#Train the model
clf.fit(X_scaled,y)
joblib.dump({'model':clf, 'scaler':scaler}, 'plastic_classifier02.pkl')
print("Model saved to plastic_classifier02.pkl")


