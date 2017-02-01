function [GlobalTime, TrackErr, TrackErrg] = InitFunction
%#codegen

% Tempo di simulazione
GlobalTime = 0; % [s] Istante di inizio simulazione.


% [rad] Inizializzazione stato interno modello di evoluzione dell'angolo di
% track.
TrackErr = 0;
TrackErrg = 0;