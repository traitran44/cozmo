#!/usr/bin/env python

##############
#### Rishabh Ranjan
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color, data
from skimage.feature import hog

class ImageClassifier:

    def __init__(self):
        self.classifier = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)

        #create one large array of image data
        data = io.concatenate_images(ic)

        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data

        feature_data = [0] * len(data)
        print("Extracting image features...")
        for x in range(len(data)):
            image = data[x]
            fd, hog_image = hog(image, orientations=8, pixels_per_cell=(16, 16),
                                cells_per_block=(1, 1), visualize=True, multichannel=True)

            # Rescale histogram for better display
            scaled_image = exposure.rescale_intensity(hog_image, in_range=(0, 10))
            feature_data[x] = [y for z in scaled_image for y in z]

        # Please do not modify the return type below
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above

        # train model and save the trained model to self.classifier

        clf = svm.SVC(gamma='scale')
        print("Training model...")
        clf.fit(train_data, train_labels)
        self.classifier = clf

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels

        clf = self.classifier
        print("Predicting labels...")
        predicted_labels = clf.predict(data)

        # Please do not modify the return type below
        return predicted_labels


def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)

    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))

    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))

    from joblib import dump
    dump(img_clf.classifier, 'clf.joblib')


if __name__ == "__main__":
    main()
