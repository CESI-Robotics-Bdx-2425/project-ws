function initializeApp() {
  const hostname = window.location.hostname;
  let ros;
  let topics = {};

  // Variables Bool
  let toggleMicValue = true;
  let partialSpan = null;
  let previousFinalSpan = null;
  let currentFinalSpan = null;
  let isSttEnabled = true;
  let isInteractStatus = false;
  let tiagoSay = null;
  let settingsClickCount = 0;
  let settingsClickTimer = null;

  // Reconnection Variables
  let reconnectionAttempts = 0;
  const maxReconnectionAttempts = 200;
  const reconnectionDelay = 5000; 
  let connectionCheckInterval = 5000;

  let isReconnecting = false;

  const elements = {
    transcriptionDiv: document.getElementById("transcription"),
    answersDiv: document.getElementById("answers-data"),
    statusDiv: document.getElementById("status-text"),
    reloadBtn: document.getElementById("reload-btn"),
    statusCircle: document.getElementById("circle"),
    toggleMic: document.getElementById("toggle-mic"),
    settingsBtn: document.getElementById("settings-btn"),
    optionsDiv: document.getElementById("options"),
    loadingAnimator: document.getElementById("loading"),
    listenerMsg: document.getElementById("listener-msg"),
    listenTrue: document.getElementById("listen-true"),
    listenFalse: document.getElementById("listen-false"),
    questionData: document.getElementById("question-data"),
    containerQuestionAnswers: document.getElementById("container-questions-anwsers"),
    retranscriptionDiv: document.getElementById("retanscript-div"),
    refillToggle: document.getElementById("toggle-refill"),
  };

  const cleanupSubscriptions = () => {
    Object.values(topics).forEach(topic => {
      if (topic && typeof topic.unsubscribe === 'function') {
        topic.unsubscribe();
      }
    });
    topics = {};
  };

  function importRoslib() {
    return new Promise((resolve, reject) => {
      const scriptUrl = `http://${hostname}:8000/roslib`;
      const script = document.createElement("script");
      script.src = scriptUrl;
      script.type = "text/javascript";

      script.onload = () => {
        console.log("Roslib chargé avec succès !");
        resolve(true);
      };

      script.onerror = () => {
        console.error("Erreur lors du chargement de Roslib.");
        reject(false);
      };

      document.head.appendChild(script);
    });
  }

  function importstyles() {
    return new Promise((resolve, reject) => {
      const scriptUrl = `http://${hostname}:8000/styles`;
      const link = document.createElement("link");
      link.href = scriptUrl;
      link.rel = "stylesheet";
      link.type = "text/css";

      link.onload = () => {
        console.log("Styles chargés avec succès !");
        resolve(true);
      };

      link.onerror = () => {
        console.error("Erreur lors du chargement des styles.");
        reject(false);
      };

      document.head.appendChild(link);
    });
  }

  function startup() {
    updateStatusDisplay("Chargement des fichiers...", false);
    Promise.all([importRoslib(), importstyles()])
      .then(() => {connectToRos();})
      .catch((error) => {
        console.error("Erreur lors du chargement des fichiers :", error);
        updateStatusDisplay("Erreur de chargement", false);
      });
  }

  const connectToRos = () => {
    cleanupSubscriptions();
    if (ros) {
      ros.close();
      ros = null;
    }
    ros = new ROSLIB.Ros({ url: `ws://${hostname}:9090` });
    updateStatusDisplay("Connexion en cours...", false);

    ros.on("connection", () => {
      console.log("Connexion ROS réussie !");
      updateReloadButtonVisibility(true);
      updateStatusDisplay("Connecté", true);
      reconnectionAttempts = 0;
      subscribeToTopics();
    });

    ros.on("error", () => {
      console.error("Erreur de connexion ROS.");
      updateReloadButtonVisibility(false);
      updateStatusDisplay("Erreur de connexion", false);
      attemptReconnection();
    });

    ros.on("close", () => {
      console.warn("Connexion ROS fermée.");
      updateReloadButtonVisibility(false);
      updateStatusDisplay("Déconnecté...", false);
      if (ros) {
        ros.close();
        ros = null;
      }
      attemptReconnection();
    });
  };

  const subscribeToTopics = () => {
    topics = {
      stt_partial: new ROSLIB.Topic({ ros, name: "/stt/partial", messageType: "std_msgs/String" }),
      stt_full: new ROSLIB.Topic({ ros, name: "/stt/full", messageType: "std_msgs/String" }),
      tiago_interact_stt_status: new ROSLIB.Topic({ ros, name: "/tiago_interact/stt/status", messageType: "std_msgs/Bool" }),
      tiago_asker_question_possible_answers: new ROSLIB.Topic({ ros, name: "/tiago_asker/question/possible_answers", messageType: "std_msgs/String" }),
      tts_goal: new ROSLIB.Topic({ ros, name: "/tts/goal", messageType: "pal_interaction_msgs/TtsActionGoal" }),
      tts_result: new ROSLIB.Topic({ ros, name: "/tts/result", messageType: "pal_interaction_msgs/TtsActionResult" }),
    };

    topics.tiago_interact_stt_status.subscribe((message) => {
      isSttEnabled = message.data;
      elements.transcriptionDiv.style.display = isSttEnabled ? "block" : "none";
      updateListenerDisplay(isSttEnabled);
    });

    topics.stt_partial.subscribe(handlePartialMessage);
    topics.stt_full.subscribe(handleFullMessage);
    topics.tiago_asker_question_possible_answers.subscribe(loadAnswersAndDisplay);
    topics.tts_goal.subscribe(loadAskTiagoRealTime);
    topics.tts_result.subscribe(onEndspeak);
  };

  const loadAskTiagoRealTime = (message) => {
    updateListenerDisplay(false);
    console.log(message.goal.rawtext.text);
    tiagoSay = document.createElement("span");
    tiagoSay.setAttribute("id", "retanscript");
    elements.questionData.appendChild(tiagoSay);
    tiagoSay.textContent = message.goal.rawtext.text;
  };

  const onEndspeak = () => {
    console.log("Retrait du message");
    const lastTiagoMessage = document.querySelector("#retanscript:last-child");
    if (lastTiagoMessage) {
      lastTiagoMessage.remove();
      updateListenerDisplay(true);
    }
  };

  const loadAnswersAndDisplay = (answers) => {
    elements.answersDiv.innerHTML = "";
    if (answers && answers.data) {
      const answerList = answers.data.split(",");
      answerList.forEach((answer) => {
        const answerElement = document.createElement("button");
        answerElement.className = "answer-element";
        answerElement.textContent = answer.trim();
        answerElement.addEventListener("click", () => {
          callAnswerService(answer.trim());
        });
        elements.answersDiv.appendChild(answerElement);
      });
    } else {
      elements.answersDiv.textContent = "Aucune réponse disponible.";
    }
  };

  const callAnswerService = (answer) => {
    const answerService = new ROSLIB.Service({
      ros: ros,
      name: `/tiago_interact/gui/answer`,
      serviceType: "tiago_interact/TiagoInteractGUI",
    });

    answerService.callService(
      new ROSLIB.ServiceRequest({
        answer: answer
      }),
      (result) => {
        console.log(`Service '${answer}' appelé avec succès`, result);
      },
      (error) => {
        console.error(`Erreur lors de l'appel du service '${answer}'`, error);
      }
    );
  };

  const attemptReconnection = () => {
    if (isReconnecting) {
      console.log("Reconnexion déjà en cours");
      return;
    }

    if (reconnectionAttempts >= maxReconnectionAttempts) {
      updateStatusDisplay("Échec de reconnexion après plusieurs tentatives.", false);
      isReconnecting = false;
      return;
    }

    isReconnecting = true;
    reconnectionAttempts++;
    console.log(`Tentative de reconnexion : ${reconnectionAttempts} / ${maxReconnectionAttempts}`);
    
    setTimeout(() => {
      connectToRos();
      isReconnecting = false;
    }, reconnectionDelay);
  };

  const updateReloadButtonVisibility = (isConnected) => {
    elements.reloadBtn.style.display = isConnected ? "none" : "block";
  };

  const updateStatusDisplay = (status, isConnected) => {
    elements.statusDiv.textContent = `Status: ${status}`;
    elements.statusCircle.style.display = isConnected ? "Grid" : "none";
    elements.loadingAnimator.style.display = isConnected ? "none" : "Grid";
    elements.reloadBtn.style.display = isConnected ? "none" : "block";
  };

  const updateListenerDisplay = (isListening) => {
    console.log("isListening : " + isListening);
    if (typeof isListening !== "boolean") {
      console.error("Erreur : La donnée passée à 'updateListenerDisplay' n'est pas un booléen.");
      elements.listenerMsg.textContent = "Erreur : État inconnu.";
      elements.listenTrue.style.display = "none";
      elements.listenFalse.style.display = "none";
      return;
    } else {
      elements.listenerMsg.textContent = isListening ? "Je t'écoute :D" : "Je ne t'écoute pas ;)";
      elements.listenTrue.style.display = isListening ? "Grid" : "none";
      elements.listenFalse.style.display = isListening ? "none" : "Grid";
    }
  };

  const handlePartialMessage = (message) => {
    if (!isSttEnabled) return;
    if (!partialSpan) {
      partialSpan = document.createElement("span");
      partialSpan.className = "partial";
      elements.transcriptionDiv.appendChild(partialSpan);
    }
    partialSpan.textContent = message.data;
  };

  const handleFullMessage = (message) => {
    if (!isSttEnabled) return;
    if (partialSpan) {
      partialSpan.remove();
      partialSpan = null;
    }
    if (currentFinalSpan) {
      if (previousFinalSpan) previousFinalSpan.remove();
      previousFinalSpan = currentFinalSpan;
      previousFinalSpan.className = "previous-final";
    }

    currentFinalSpan = document.createElement("span");
    currentFinalSpan.className = "final";
    currentFinalSpan.textContent = message.data;
    elements.transcriptionDiv.appendChild(currentFinalSpan);
  };

  document.addEventListener("DOMContentLoaded", () => {
    elements.reloadBtn.addEventListener("click", connectToRos);
    
    elements.toggleMic.addEventListener("click", () => {
      elements.toggleMic.disabled = true;
      setTimeout(() => (elements.toggleMic.disabled = false), 3000);
      toggleMicValue = !toggleMicValue;
      console.log("toggleMicValue : " + toggleMicValue);
      elements.toggleMic.style.backgroundColor = toggleMicValue ? "lime" : "red";
      elements.toggleMic.style.color = toggleMicValue ? "black" : "white";
      elements.retranscriptionDiv.style.display = toggleMicValue ? "block" : "none";

      const micService = new ROSLIB.Service({
        ros: ros,
        name: "/tiago_interact/stt/toggle",
        serviceType: "std_srvs/Empty",
      });

      micService.callService(
        new ROSLIB.ServiceRequest(),
        () => console.log("Service appelé avec succès")
      );
    });
    
    elements.settingsBtn.addEventListener("click", () => {
      elements.optionsDiv.style.display = elements.optionsDiv.style.display === "none" ? "flex" : "none";
      elements.containerQuestionAnswers.style.display = elements.containerQuestionAnswers.style.display === "none" ? "flex" : "none";
      settingsClickCount++;
      
      if (settingsClickTimer) {
        clearTimeout(settingsClickTimer);
      }
      
      settingsClickTimer = setTimeout(() => {
        settingsClickCount = 0;
      }, 3000);
      
      if (settingsClickCount >= 5) {
        elements.optionsDiv.style.display = elements.optionsDiv.style.display === "none" ? "flex" : "none";
        settingsClickCount = 0;
      }
    });
    
    elements.refillToggle.addEventListener("click", () => {
      elements.refillToggle.style.backgroundColor = "orange";
      
      const refillService = new ROSLIB.Service({
        ros: ros,
        name: "/refill",
        serviceType: "std_srvs/Empty",
      });

      refillService.callService(
        new ROSLIB.ServiceRequest(),
        () => {
          console.log("Service appelé avec succès");
          elements.refillToggle.style.backgroundColor = "lime";
        }
      );
    });
  });

  startup();
}

initializeApp();