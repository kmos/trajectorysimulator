data = normrnd(0,1,1000,2);
labels = ones(length(data),1);

% Construct one-class SVM with RDF kernel (Gaussian)
model = svmtrain(labels, data, '-s 2 -t 2 ');
%-n 0.001 -g 0.001

% Use the same data for label prediction
[predicted_labels] = svmpredict(labels, data, model);
inside_indices = find(predicted_labels > 0);

figure; hold on;
% Scatterplot of all data, blue circles
scatter(data(:,1), data(:,2), 30, 'blue');

% Scatterplot of all support vectors, small red circles
scatter(model.SVs(:,1), model.SVs(:,2), 20, 'red');

% Scatterplot of all data inside the one-classs, small green circles
scatter(data(inside_indices,1), data(inside_indices,2), 10, 'green');