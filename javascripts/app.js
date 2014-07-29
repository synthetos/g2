'use strict';

// Declare app level module which depends on filters, and services
var binaryApp = angular.module('binaryApp', []);

binaryApp.controller('BinaryCtrl', function ($scope, $http) {
  $scope.binaries = [];
  
  $http.get('./tinyg-binaries.json').success(function(data) {
    $scope.binaries = data.binaries;
  });
});
