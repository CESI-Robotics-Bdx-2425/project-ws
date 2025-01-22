function leJcleS() {

  let hostname = window.location.hostname;

  let ros;

  // Variables Bool
  let toggleMicValue = true;
  let partialSpan = null;
  let previousFinalSpan = null;
  let currentFinalSpan = null;
  let isSttEnabled = true;
  let isInteractStatus = false;
  let tiagoSay = null;

  // Reconnexion Variables
  let reconnectionAttempts = 0; // Nombre de tentatives de reconnexion
  const maxReconnectionAttempts = 200; // Nombre maximal de tentatives
  const reconnectionDelay = 20000; // Délai entre les tentatives (en millisecondes)
  let countdownInterval = 20; // Pour stocker l'intervalle de décompte
  let connectionCheckInterval = 5000; // Intervalle de vérification de connexion

  function startup() {
    updateStatusDisplay("Chargement des fichiers...", false);
    Promise.all([importRoslib(), importstyles()])
      .then(() => {
        connectToRos(); // Connexion ROS après le chargement des fichiers
      })
      .catch((error) => {
        console.error("Erreur lors du chargement des fichiers :", error);
      });
  }

  function importRoslib() {
    return new Promise((resolve, reject) => {
      // Récupérer le hostname de manière dynamique
      const scriptUrl = `http://${hostname}:8000/roslib`;

      // Créer une balise <script>
      const script = document.createElement("script");
      script.src = scriptUrl;
      script.type = "text/javascript";

      // Ajouter des gestionnaires pour le chargement et les erreurs
      script.onload = () => {
        console.log("Roslib chargé avec succès !");
        resolve(true); // Retourne true si le script se charge correctement
      };

      script.onerror = () => {
        console.error("Erreur lors du chargement de Roslib.");
        reject(false); // Retourne false en cas d'erreur
      };

      // Ajouter le script dans la balise <head>
      document.head.appendChild(script);
    });
  }

  function importstyles() {
    return new Promise((resolve, reject) => {
      // Récupérer le hostname de manière dynamique
      const scriptUrl = `http://${hostname}:8000/styles`;

      // Créer une balise <style>
      const script = document.createElement("link");
      script.href = scriptUrl;
      script.rel = "stylesheet";
      script.type = "text/css";

      // Ajouter des gestionnaires pour le chargement et les erreurs
      script.onload = () => {
        console.log("Styles chargés avec succès !");
        resolve(true); // Retourne true si le style se charge correctement
      };

      script.onerror = () => {
        console.error("Erreur lors du chargement des styles.");
        reject(false); // Retourne false en cas d'erreur
      };

      // Ajouter le script dans la balise <head>
      document.head.appendChild(script);
    });
  }

  // Variables globales
  const elements = {
    transcriptionDiv: document.getElementById("transcription"),
    answersDiv: document.getElementById("anwsers"),
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
  };

  // Connexion ROS et abonnements
  const connectToRos = () => {
    ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

    updateStatusDisplay("Connexion en cours...", false);

    ros.on("connection", () => {
      console.log("Connexion ROS réussie !");
      updateReloadButtonVisibility(true);
      updateStatusDisplay("Connecté", true);
      reconnectionAttempts = 0; // Réinitialiser les tentatives de reconnexion
    });

    ros.on("error", () => {
      console.error("Erreur de connexion ROS.");
      updateReloadButtonVisibility(false);
      updateStatusDisplay("Erreur de connexion", false);
      attemptReconnection(); // Lancer une tentative de reconnexion
    });

    ros.on("close", () => {
      console.warn("Connexion ROS fermée.");
      updateReloadButtonVisibility(false);
      updateStatusDisplay("Déconnecté...", false);
      attemptReconnection(); // Lancer une tentative de reconnexion
    });

    const topics = {
      stt_partial: new ROSLIB.Topic({
        ros,
        name: "/stt/partial",
        messageType: "std_msgs/String",
      }),
      stt_full: new ROSLIB.Topic({
        ros,
        name: "/stt/full",
        messageType: "std_msgs/String",
      }),
      tiago_interact_stt_status: new ROSLIB.Topic({
        ros,
        name: "/tiago_interact/stt/status",
        messageType: "std_msgs/Bool",
      }),
      tiago_asker_question_possible_answers: new ROSLIB.Topic({
        ros,
        name: "/tiago_asker/question/possible_answers",
        messageType: "std_msgs/String",
      }),
      tts_goal: new ROSLIB.Topic({
        ros,
        name: "/tts/goal",
        messageType: "pal_interaction_msgs/TtsActionGoal",
      }),
      tts_result: new ROSLIB.Topic({
        ros,
        name: "/tts/result",
        messageType: "pal_interaction_msgs/TtsActionResult",
      }),
    };

    topics.tiago_interact_stt_status.subscribe((message) =>
      updateListenerDisplay(message.data)
    );

    topics.tiago_interact_stt_status.subscribe((message) => {
      isSttEnabled = message.data;
      elements.transcriptionDiv.style.display = isSttEnabled ? "block" : "none";
      elements.answersDiv.style.display = isSttEnabled ? "none" : "block";
    });

    topics.stt_partial.subscribe(handlePartialMessage);

    topics.stt_full.subscribe(handleFullMessage);

    // subscribe au topic de Question / Réponse
    topics.tiago_asker_question_possible_answers.subscribe(
      loadAnswersAndDisplay
    );
    topics.tts_goal.subscribe(loadAskTiagoRealTime);
    topics.tts_result.subscribe(onEndspeak);
  };

  // Gestions du tts et des Réponces
  const loadAskTiagoRealTime = (message) => {
    updateListenerDisplay(false);
    console.log(message.goal.rawtext.text);
    tiagoSay = document.createElement("span");
    tiagoSay.setAttribute("id", "retanscript");
    elements.questionData.appendChild(tiagoSay);
    tiagoSay.textContent = message.goal.rawtext.text;
  };

  // Détections quand Tiago a fini de parler
  const onEndspeak = (message) => {
    console.log("Retrait du message");

    // Rechercher le dernier message ajouté pour Tiago et le retirer
    const lastTiagoMessage = document.querySelector("#retanscript:last-child");

    if (lastTiagoMessage) {
      // Si un message de Tiago est présent, le retirer du DOM
      lastTiagoMessage.remove();
      updateListenerDisplay(true);
    }
  };

  // Affichage des réponces possible
  const loadAnswersAndDisplay = (anwsers) => {};

  const startConnectionCheck = () => {
    connectionCheckInterval = setInterval(() => {
      // Vérifie si la connexion est toujours active
      if (!ros || !ros.isConnected) {
        console.warn("Connexion ROS perdue. Tentative de reconnexion...");
        let msg =
          "Tentative : " +
          reconnectionAttempts +
          " / " +
          maxReconnectionAttempts;
        updateStatusDisplay(msg, false);
        attemptReconnection(); // Tentative de reconnexion si la connexion est perdue
      } else {
        console.log("Vérification de connexion : Connecté.");
      }
    }, connectionCheckInterval); // Vérification toutes les 5 secondes
  };

  // Arrêter la vérification de connexion
  const stopConnectionCheck = () => {
    clearInterval(connectionCheckInterval);
  };

  // Tentative de reconnexion avec une limite
  const attemptReconnection = () => {
    if (reconnectionAttempts >= maxReconnectionAttempts) {
      updateStatusDisplay(
        "Échec de reconnexion après plusieurs tentatives.",
        false
      );
      stopConnectionCheck(); // Arrêter la vérification si le max est atteint
      return;
    }

    reconnectionAttempts++;
    console.log(
      "reconnectionAttempts : " +
        reconnectionAttempts +
        " / maxReconnectionAttempts : " +
        maxReconnectionAttempts
    );

    // Définir un délai avant la prochaine tentative de reconnexion
    setTimeout(() => {
      connectToRos();
    }, reconnectionDelay); // Essayer à nouveau après 20 secondes
  };

  // Affichage / masquage du bouton Reconnexion
  const updateReloadButtonVisibility = (isConnected) => {
    elements.reloadBtn.style.display = isConnected ? "none" : "block";
  };

  // Mise à jour des éléments d'état
  const updateStatusDisplay = (status, isConnected) => {
    elements.statusDiv.textContent = `Status: ${status}`;
    elements.statusCircle.style.display = isConnected ? "grid" : "none";
    elements.loadingAnimator.style.display = isConnected ? "none" : "grid";
    elements.reloadBtn.style.display = isConnected ? "none" : "block";
  };

  // Mise à jour de l'écouteur
  const updateListenerDisplay = (isListening) => {
    console.log("isListening : " + isListening);
    if (typeof isListening !== "boolean") {
      console.error(
        "Erreur : La donnée passée à 'updateListenerDisplay' n'est pas un booléen."
      );
      elements.listenerMsg.textContent = "Erreur : État inconnu.";
      elements.listenTrue.style.display = "none";
      elements.listenFalse.style.display = "none";
      return; // Stoppe l'exécution
    } else {
      elements.listenerMsg.textContent = isListening
        ? "Je t'écoute :D"
        : "Je ne t'écoute pas ;)";
      elements.listenTrue.style.display = isListening ? "grid" : "none";
      elements.listenFalse.style.display = isListening ? "none" : "grid";
    }
  };

  // Gestion des messages STT
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
      previousFinalSpan.className = "partial";
    }

    // Ajouter un <br> si une condition est remplie
    if (
      /* votre condition ici, par exemple, si ce n'est pas la première ligne */ elements
        .transcriptionDiv.children.length > 0
    ) {
      const lineBreak = document.createElement("br");
      elements.transcriptionDiv.appendChild(lineBreak);
    }

    currentFinalSpan = document.createElement("span");
    currentFinalSpan.className = "final";
    currentFinalSpan.textContent = message.data;
    elements.transcriptionDiv.appendChild(currentFinalSpan);
  };

  // Gestion des événements
  elements.reloadBtn.addEventListener("click", connectToRos);

  elements.toggleMic.addEventListener("click", () => {
    elements.toggleMic.disabled = true;
    setTimeout(() => (elements.toggleMic.disabled = false), 3000);

    toggleMicValue = !toggleMicValue;
    elements.toggleMic.style.backgroundColor = toggleMicValue ? "lime" : "red";
    elements.toggleMic.style.color = toggleMicValue ? "black" : "white";

    const micService = new ROSLIB.Service({
      ros: new ROSLIB.Ros(),
      name: "/tiago_interact/stt/toggle",
      serviceType: "std_srvs/Empty",
    });

    micService.callService(new ROSLIB.ServiceRequest(), () =>
      console.log("Service appelé avec succès")
    );
  });

  elements.settingsBtn.addEventListener("click", () => {
    elements.optionsDiv.style.display =
      elements.optionsDiv.style.display === "none" ? "block" : "none";
  });

  // Lancer la connexion
  startup();
};
// quand le fichier est chargé, on lance le script
leJcleS();