% ScenarioSimulation.m
%
% This script configures a physical layer security scenario with a single
% base station, two relays (one car-based and one UAV-based), three
% legitimate users, and two eavesdroppers. The geometry is explicitly
% modelled to capture the different propagation conditions of terrestrial
% and aerial links. For each legitimate user, the script computes the
% end-to-end SNR obtained through decode-and-forward relaying (selecting
% the best relay) and contrasts it against the strongest eavesdropping
% SNR. The resulting secrecy rates are summarised in a table for quick
% inspection.
%
% The configuration reflects the updated scenario discussed in the
% project documentation. In particular, all UAV nodes fly at 150 m.

clear; close all; clc;

%% Scenario description ---------------------------------------------------
scenario.baseStation = struct( ...
    'name', "Base Station", ...
    'type', "base", ...
    'position', [0 0 25]); % Small tower height to avoid ground clutter

uavHeight = 150; % metres

scenario.relays = struct( ...
    'name', {"Car Relay", "UAV Relay"}, ...
    'type', {"car", "uav"}, ...
    'position', {[220 60 0], [-120 340 uavHeight]});

scenario.legitimateUsers = struct( ...
    'name', {"Car User", "UAV User 1", "UAV User 2"}, ...
    'type', {"car", "uav", "uav"}, ...
    'position', { [640 -40 0], [180 420 uavHeight], [420 -260 uavHeight] });

scenario.eavesdroppers = struct( ...
    'name', {"Car Eavesdropper", "UAV Eavesdropper"}, ...
    'type', {"car", "uav"}, ...
    'position', {[360 180 0], [-220 260 uavHeight]});

%% Propagation and transceiver assumptions --------------------------------
params.referenceDistance = 1; % metres
params.noisePower_dBm = -96;  % Thermal noise with modest receiver NF
params.transmitPower_dBm.base = 40; % 10 W EIRP for the base station
params.transmitPower_dBm.carRelay = 33; % 2 W for car relay
params.transmitPower_dBm.uavRelay = 30; % 1 W for UAV relay
params.pathLoss.groundExponent = 2.7; % Urban vehicular environment
params.pathLoss.mixedExponent = 2.3; % Ground-to-air or air-to-ground
params.pathLoss.aerialExponent = 2.0; % Air-to-air links (LoS dominated)

%% Pre-compute noise and transmit power in linear scale -------------------
noisePower = db2pow(params.noisePower_dBm - 30); % Convert dBm to Watts

relayPowers = containers.Map;
relayPowers("car") = db2pow(params.transmitPower_dBm.carRelay - 30);
relayPowers("uav") = db2pow(params.transmitPower_dBm.uavRelay - 30);

basePower = db2pow(params.transmitPower_dBm.base - 30);

%% Evaluate SNR for each legitimate user ----------------------------------
numUsers = numel(scenario.legitimateUsers);
userSummaries = repmat(struct('name',"",'bestRelay',"",'mainSNR',0,'eavesSNR',0,'secrecyRate',0), numUsers, 1);

for idxUser = 1:numUsers
    user = scenario.legitimateUsers(idxUser);

    bestRelayName = "";
    bestMainSNR = 0;
    bestEavesSNR = 0;

    for idxRelay = 1:numel(scenario.relays)
        relay = scenario.relays(idxRelay);
        relayPower = relayPowers(relay.type);

        % Base station to relay hop
        snrFirstHop = hopSNR(scenario.baseStation, relay, basePower, noisePower, params);

        % Relay to user hop
        snrSecondHop = hopSNR(relay, user, relayPower, noisePower, params);

        % Decode-and-forward end-to-end SNR is limited by the weakest hop
        mainSNR = min(snrFirstHop, snrSecondHop);

        % Eavesdroppers listen to the relay transmission.
        eavesSNR = 0;
        for idxEve = 1:numel(scenario.eavesdroppers)
            eve = scenario.eavesdroppers(idxEve);
            eavesSNR = max(eavesSNR, hopSNR(relay, eve, relayPower, noisePower, params));
        end

        if mainSNR > bestMainSNR
            bestMainSNR = mainSNR;
            bestEavesSNR = eavesSNR;
            bestRelayName = relay.name;
        end
    end

    secrecyRate = max(log2(1 + bestMainSNR) - log2(1 + bestEavesSNR), 0);

    userSummaries(idxUser).name = user.name;
    userSummaries(idxUser).bestRelay = bestRelayName;
    userSummaries(idxUser).mainSNR = bestMainSNR;
    userSummaries(idxUser).eavesSNR = bestEavesSNR;
    userSummaries(idxUser).secrecyRate = secrecyRate;
end

%% Present the results ----------------------------------------------------
userNames = {userSummaries.name}';
bestRelayNames = {userSummaries.bestRelay}';
mainSNRdB = linear2db([userSummaries.mainSNR]);
eavesSNRdB = linear2db([userSummaries.eavesSNR]);
secrecyRates = [userSummaries.secrecyRate]';

resultsTable = table(userNames, bestRelayNames, mainSNRdB', eavesSNRdB', secrecyRates, ...
    'VariableNames', {'User', 'SelectedRelay', 'MainSNR_dB', 'EavesdropperSNR_dB', 'SecrecyRate_bpsHz'});

disp('Updated scenario results (best relay per legitimate user):');
disp(resultsTable);

%% Helper functions -------------------------------------------------------
function snrValue = hopSNR(txNode, rxNode, transmitPower, noisePower, params)
    d3d = pointDistance(txNode.position, rxNode.position);
    plExponent = selectPathlossExponent(txNode.position(3), rxNode.position(3), params.pathLoss);
    pathGain = linkGain(d3d, params.referenceDistance, plExponent);
    snrValue = transmitPower * pathGain / noisePower;
end

function gain = linkGain(distance, referenceDistance, exponent)
    effectiveDistance = max(distance, referenceDistance);
    gain = (referenceDistance ./ effectiveDistance) .^ exponent;
end

function exponent = selectPathlossExponent(heightTx, heightRx, pathLoss)
    groundThreshold = 1.5; % Anything below this is considered terrestrial
    isTxAir = heightTx > groundThreshold;
    isRxAir = heightRx > groundThreshold;

    if ~isTxAir && ~isRxAir
        exponent = pathLoss.groundExponent;
    elseif isTxAir && isRxAir
        exponent = pathLoss.aerialExponent;
    else
        exponent = pathLoss.mixedExponent;
    end
end

function d = pointDistance(p1, p2)
    diffVec = p1 - p2;
    d = sqrt(sum(diffVec.^2));
end

function value_dB = linear2db(valueLinear)
    value_dB = 10 * log10(valueLinear);
end

function valueLinear = db2pow(value_dB)
    valueLinear = 10.^(value_dB/10);
end
