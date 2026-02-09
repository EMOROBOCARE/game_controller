I want you to adapt my code to follow the game structure in colours.
Keep the same structure and style as in the colours game for the rest of the games.
There are three dificulties: basic, intermediate and advanced.
Code and comments are in english, but the game content is in spanish.
I also want to unify phase 4 and 5 into a single phase where we sample one correct and incorrect answer randomly and we first ask for the incorrect one and then for the correct one.
For the legacy_ros integration take into accunt that communication_hub may translate some messages into intents.
Integrate all legacy topics into my app accordingly in a clean way.
Make code organised with no file over 200 lines and grouping related functionalities together and folders conceptually.
Create data types for everything using pydantic models.
Integrte with ros2 using rclpy.
---
For basic difficulty there will be 2 options, intermediate 3 and advance 4 options.

 ## Phase 1:
 Renders GameScreen (with matchingRound payload)
 matchingRound = [
  {answer1},...,{answerN}
 ]

 ## Phase 2:
 GameScreen with answers=False so the answer don't render

 ## Phase 3: 
 Discriminate between different options (depending on difficulty), these questions will be set in the game config (that can override phase configs)

  ## Phase 4:
  The only answers are Yes/No using the same AnswerComponent.
  Correct answer is always no.

  ## Phase 5:
  The only answers are Yes/No using the same AnswerComponent. Correct answer is always yes.

  ## Phase 6: 
The child will ask where is between a set of options (depending on difficulty) there is no correct answer, the controller just set the non-said as hidden and the said ones as highlighted and says: "Aquííí" in a happy way.

  ## Phase 7:
  The robot will ask the child "Es un ... o ..." en nivel básico la respuesta correcta será la segunda, en intermedio la primera y en avanzado aleatorio.
