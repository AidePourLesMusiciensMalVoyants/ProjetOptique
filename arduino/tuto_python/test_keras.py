# Import pandas
# Keras Tutorial: Deep Learning in Python
# https://www.datacamp.com/community/tutorials/deep-learning-python#comments

# This tutorial is the one found on Datacamp (see link)
# I just added some comments and 2/3 compatibility modifications

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import seaborn as sns

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

from keras.models import Sequential
from keras.layers import Dense

##### white = pd.read_csv("http://archive.ics.uci.edu/ml/machine-learning-databases/wine-quality/winequality-white.csv", sep=';')
white = pd.read_csv("winequality-white.csv", sep=';')

# Read in red wine data
##### red = pd.read_csv("http://archive.ics.uci.edu/ml/machine-learning-databases/wine-quality/winequality-red.csv", sep=';')
red = pd.read_csv("winequality-red.csv", sep=';')

# Print info on white wine
print(white.info())

# Print info on red wine
print(red.info())

# First rows of `red`
red.head()

# Last rows of `white`
white.tail()

# Take a sample of 5 rows of `red`
red.sample(5)

# Describe `white`
white.describe()

# Double check for null values in `red`
pd.isnull(red)

# https: // docs.scipy.org / doc / numpy / reference / generated / numpy.histogram.html

# https://en.wikipedia.org/wiki/Histogram
# To construct a histogram, the first step is to "bin" the range of values—that is,
# divide the entire range of values into a series of intervals—and then count
# how many values fall into each interval.
# The bins are usually specified as consecutive, non-overlapping intervals of a variable.
# The bins (intervals) must be adjacent, and are often (but are not required to be) of equal size

# bins : int or sequence of scalars or str, optional
# If bins is an int, it defines the number of equal-width bins in the given
# range (10, by default).
# If bins is a sequence, it defines the bin edges, including the rightmost edge,
# allowing for non-uniform bin widths.

print(np.histogram(red.alcohol, bins=[7,8,9,10,11,12,13,14,15]))

print(np.histogram(white.alcohol, bins=[7,8,9,10,11,12,13,14,15]))


fig, ax = plt.subplots(1, 2, figsize=(8, 4))

ax[0].scatter(red['quality'], red["sulphates"], color="red")
ax[1].scatter(white['quality'], white['sulphates'], color="white", edgecolors="black", lw=0.5)

ax[0].set_title("Red Wine")
ax[1].set_title("White Wine")
ax[0].set_xlabel("Quality")
ax[1].set_xlabel("Quality")
ax[0].set_ylabel("Sulphates")
ax[1].set_ylabel("Sulphates")
ax[0].set_xlim([0,10])
ax[1].set_xlim([0,10])
ax[0].set_ylim([0,2.5])
ax[1].set_ylim([0,2.5])
fig.subplots_adjust(wspace=0.5)
fig.suptitle("Wine Quality by Amount of Sulphates")

plt.show()

# -------------------------------------------------------------
# Acidity

# Apart from the sulphates, the acidity is one of the major and important wine characteristics
# that is necessary to achieve quality wines.
# Great wines often balance out acidity, tannin, alcohol and sweetness.
# Some more research taught me that in quantities of 0.2 to 0.4 g/L, volatile acidity
# doesn’t affect a wine’s quality.
# At higher levels, however, volatile acidity can give wine a sharp, vinegary tactile sensation.
# Extreme volatile acidity signifies a seriously flawed wine.
# Let’s put the data to the test and make a scatter plot that plots the alcohol versus the
# volatile acidity. The data points should be colored according to their rating or quality label:

np.random.seed(570)

redlabels = np.unique(red['quality'])
whitelabels = np.unique(white['quality'])

fig, ax = plt.subplots(1, 2, figsize=(8, 4))
redcolors = np.random.rand(6, 4)
whitecolors = np.append(redcolors, np.random.rand(1, 4), axis=0)

for i in range(len(redcolors)):
    redy = red['alcohol'][red.quality == redlabels[i]]
    redx = red['volatile acidity'][red.quality == redlabels[i]]
    ax[0].scatter(redx, redy, c=redcolors[i])
for i in range(len(whitecolors)):
    whitey = white['alcohol'][white.quality == whitelabels[i]]
    whitex = white['volatile acidity'][white.quality == whitelabels[i]]
    ax[1].scatter(whitex, whitey, c=whitecolors[i])

ax[0].set_title("Red Wine")
ax[1].set_title("White Wine")
ax[0].set_xlim([0, 1.7])
ax[1].set_xlim([0, 1.7])
ax[0].set_ylim([5, 15.5])
ax[1].set_ylim([5, 15.5])
ax[0].set_xlabel("Volatile Acidity")
ax[0].set_ylabel("Alcohol")
ax[1].set_xlabel("Volatile Acidity")
ax[1].set_ylabel("Alcohol")
# ax[0].legend(redlabels, loc='best', bbox_to_anchor=(1.3, 1))
ax[1].legend(whitelabels, loc='best', bbox_to_anchor=(1.3, 1))
# fig.suptitle("Alcohol - Volatile Acidity")
fig.subplots_adjust(top=0.85, wspace=0.7)

plt.show()
exit()



# Preprocess Data

# Now that you have explored your data, it’s time to act upon the insights that you have gained!
# Let’s preprocess the data so that you can start building your own neural network!

# Add `type` column to `red` with value 1
red['type'] = 1

# Add `type` column to `white` with value 0
white['type'] = 0

# Append `white` to `red`
wines = red.append(white, ignore_index=True)

# You set ignore_index to True in this case because you don’t want to keep the index
# labels of white when you’re appending the data to red: you want the labels to continue from where
# they left off in red, not duplicate index labels from joining both data sets together.

red.head()

# Intermezzo: Correlation Matrix

# Now that you have the full data set, it’s a good idea to also do a quick data exploration;
# You already know some stuff from looking at the two data sets separately, and now it’s time to
# gather some more solid insights, perhaps.
# Since it can be somewhat difficult to interpret graphs, it’s also a good idea to plot a correlation matrix.
# This will give insights more quickly about which variables correlate:

# -------------------------------------------------------------

corr = wines.corr()
sns.heatmap(corr,
            xticklabels=corr.columns.values,
            yticklabels=corr.columns.values)

# sns.plt.show() - cette ligne inutile ne marche pas
# As you would expect, there are some variables that correlate, such as density and residual sugar.
# Also volatile acidity and type are more closely connected than you originally could have guessed
# by looking at the two data sets separately, and it was kind of to be expected that free sulfur dioxide
# and total sulfur dioxide were going to correlate.

# -------------------------------------------------------------
# Train and Test Sets

# Imbalanced data typically refers to a problem with classification problems where the classes are
# not represented equally.Most classification data sets do not have exactly equal number of instances
# in each class, but a small difference often does not matter.
# You thus need to make sure that all two classes of wine are present in the training model.
# What’s more, the amount of instances of all two wine types needs to be more or less equal so that you do
# not favour one or the other class in your predictions.

# Import `train_test_split` from `sklearn.model_selection`
# Train and Test Sets
# Specify the data

# .ix is deprecated. Please use
# The recommended methods of indexing are:
# .loc if you want to label index
# .iloc if you want to positionally index.

# For now, import the train_test_split from sklearn.model_selection and assign the data and the target
# labels to the variables X and y. You’ll see that you need to flatten the array of target labels in order
# to be totally ready to use the X and y variables as input for the train_test_split() function.

X=wines.iloc[:,0:11]

# Specify the target labels and flatten the array
# Return a contiguous flattened array.
# A 1-D array, containing the elements of the input, is returned. A copy is made only if needed.
y=np.ravel(wines.type)

# Split the data up in train and test sets
# http://scikit-learn.org/stable/modules/generated/sklearn.model_selection.train_test_split.html
# random_state : int, RandomState instance or None, optional (default=None)
# If int, random_state is the seed used by the random number generator;
# If RandomState instance, random_state is the random number generator;
# If None, the random number generator is the RandomState instance used by np.random.

# test_size : float, int, None, optional
# If float, should be between 0.0 and 1.0 and represent the proportion of the dataset to include in the test split.
# If int, represents the absolute number of test samples.
# If None, the value is set to the complement of the train size.
# By default, the value is set to 0.25. The default will change in version 0.21.
# It will remain 0.25 only if train_size is unspecified, otherwise it will complement the specified train_size.

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.33, random_state=42)

wines.head()


X_test.head()

# -------------------------------------------------------------
# Standardize The Data
#
# Import `StandardScaler` from `sklearn.preprocessing`

# Standardization is a way to deal with these values that lie so far apart.
# The scikit-learn package offers you a great and quick way of getting your data standardized:
# import the StandardScaler module from sklearn.preprocessing and you’re ready to scale your train and test data!

# Define the scaler
# Standardize features by removing the mean and scaling to unit variance
# http://scikit-learn.org/stable/modules/generated/sklearn.preprocessing.StandardScaler.html
# fit(X, y=None)
#  : {array-like, sparse matrix}, shape [n_samples, n_features]
# The data used to compute the mean and standard deviation used for later scaling along the features axis.
scaler = StandardScaler().fit(X_train)

# Scale the train set
# transform(X, y=’deprecated’, copy=None)
# http://scikit-learn.org/stable/modules/generated/sklearn.preprocessing.StandardScaler.html
# sklearn.preprocessing.StandardScaler.fit
# Perform standardization by centering and scaling
# Il y a aussi fit_transform(X, y=None, **fit_params)

X_train = scaler.transform(X_train)

# Pourquoi ne pas avoir fait :
# fit_transform(X, y=None, **fit_params)
# Fit to data, then transform it.
# Fits transformer to X and y with optional parameters fit_params and returns a transformed version of X.
# X_train = fit_transform(X_train)

# Scale the test set
X_test = scaler.transform(X_test)



X_train.shape

#
# -------------------------------------------------------------
# Model Data

# Import `Sequential` from `keras.models`
# Import `Dense` from `keras.layers`

# Initialize the constructor
# Now how do you start building your multi-layer perceptron?
# A quick way to get started is to use the Keras Sequential model: it’s a linear stack of layers.
# You can easily create the model by passing a list of layer instances to the constructor,
# which you set up by running model = Sequential().

model = Sequential()

# Add an input layer
# When you’re making your model, it’s therefore important to take into account that your first
# layer needs to make the input shape clear.
# The model needs to know what input shape to expect and that’s why you’ll always find the input_shape,
# input_dim, input_length, or batch_size arguments in the documentation of the layers and in practical
# application of those layers.

# In this case, you will have to use a Dense layer, which is a fully connected layer.
# Dense layers implement the following operation: output = activation(dot(input, kernel) + bias).
# Note that without the activation function, your Dense layer would consist only of two linear
# operations: a dot product and an addition.

# La forme (shape) d'un tenseur est le nombre d'éléments dans chaque dimension.

# What flows between layers are tensors. Tensors can be seen as matrices, with shapes.
# In Keras, the input layer itself is not a layer, but a tensor.
# It's the starting tensor you send to the first hidden layer.
# This tensor must have the same shape as your training data.

# https://stackoverflow.com/questions/44747343/keras-input-explanation-input-shape-units-batch-size-dim-etc

# Example: if you have 30 images of 50x50 pixels in RGB (3 channels), the shape of your input data is (30,50,50,3).
# Then your input layer tensor, must have this shape (see details in the "shapes in keras" section).
# Each type of layer requires the input with a certain number of dimensions.
# Dense layers require inputs as (Batch_size, input_size), 2D convolutional layers need inputs
# as (batch_size,imageside1, imageside2, channels).
# Now, the input shape is the only one you must define, because your model cannot know it. Only you know that, based on your training data.
# All the other shapes are calculated automatically based on the units and particularities of each layer.

model.add(Dense(12, activation='relu', input_shape=(11,)))

# Add one hidden layer
model.add(Dense(8, activation='relu'))

# Add an output layer
model.add(Dense(1, activation='sigmoid'))

# -------------------------------------------------------------
# https://keras.io/models/about-keras-models/#about-keras-models
# Model output shape
model.output_shape

# Model summary
# model.summary(): prints a summary representation of your model. Shortcut for utils.print_summary
model.summary()

# Model config
# model.get_config(): returns a dictionary containing the configuration of the model.

# model.get_config()

# List all weight tensors
# model.get_weights(): returns a list of all weight tensors in the model, as Numpy arrays.

# model.get_weights()

# -------------------------------------------------------------
# Compile and Fit

# The optimizer and the loss are two arguments that are required if you want to compile the model.
# Some of the most popular optimization algorithms used are the Stochastic Gradient Descent (SGD),
# ADAM and RMSprop. Depending on whichever algorithm you choose, you’ll need to tune certain parameters,
# such as learning rate or momentum.
# The choice for a loss function depends on the task that you have at hand: for example,
# for a regression problem, you’ll usually use the Mean Squared Error (MSE).
# As you see in this example, you used binary_crossentropy for the binary classification problem of
# determining whether a wine is red or white.

# After, you can train the model for 20 epochs or iterations over all the samples in X_train and y_train,
# in batches of 1 sample. You can also specify the verbose argument.
# By setting it to 1, you indicate that you want to see progress bar logging.

# In other words, you have to train the model for a specified number of epochs or exposures to the training dataset.
# An epoch is a single pass through the entire training set, followed by testing of the verification set.
# The batch size that you specify in the code above defines the number of samples that going to be propagated
# through the network.
# Also, by doing this, you optimize the efficiency because you make sure that you don’t load too many
# input patterns into memory at the same time.


model.compile(loss='binary_crossentropy',
              optimizer='adam',
              metrics=['accuracy'])

model.fit(X_train, y_train, epochs=20, batch_size=1, verbose=1)

# Predict Values
y_pred = model.predict(X_test)

y_pred[:10]

# Etant donné que la prédiction voulue est une classe et non une probabilité, il ne faut pas utiliser
# y_pred = model.predict(X_test)
# mais le code ci-dessous, qui retourne l'appartenance à une des deux classes (blanc, rouge)
y_classes = model.predict_classes(X_test)

y_classes[:5]



y_test[:5]


# Evaluate Model
score = model.evaluate(X_test, y_test,verbose=1)
print(score)

# Import the modules from `sklearn.metrics`
from sklearn.metrics import confusion_matrix, precision_score, recall_score, f1_score, cohen_kappa_score

# Confusion matrix
y_test
# La matrice de confusion doit se calculer non pas avec les probabilités mais avec les classements
# Il faut modifier la ligne du tutoriel
# confusion_matrix(y_test, y_pred)
confusion_matrix(y_test, y_classes)

y_pred[0:]

# Precision
precision_score(y_test, y_classes)

# Recall
recall_score(y_test, y_classes)

# F1 score
f1_score(y_test,y_classes)

# Cohen's kappa
cohen_kappa_score(y_test, y_classes)

