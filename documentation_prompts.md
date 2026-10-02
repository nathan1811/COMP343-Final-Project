# Development Prompt Examples

This file contains representative examples of prompts used during development of the Autonomous Drone Delivery project. The prompts below focus on debugging, integration, testing, design refinement, and documentation.

## 1. Changing the project domain

> “I want to change the project from EV charging to drone delivery, with the drones delivering only to the airport or supermarket. Keep the existing behaviour of the system while adapting the terminology and domain.”

## 2. Updating model terminology

> “Change the EV-related terminology to drones and change chargers to charging pads, while keeping the existing functionality and structure.”

## 3. Debugging drone selection

> “The prompt says ‘Take Drone-01 to the supermarket’, but the system is selecting the wrong drone. Can you help me identify what could be causing this behaviour?”

## 4. Multi-drone command handling

> “I want one natural-language command to be able to assign different destinations to multiple named drones. Check the surrounding integration and identify anything that could prevent the assignments from being preserved correctly.”

## 5. Multi-drone collision behaviour

> “Two drones are ending up on the same tile at the same time. Help me find where the scheduling/execution logic could be allowing this to happen.”

## 6. Waiting behaviour

> “If two drones would conflict, I want the higher-battery drone to wait while the lower-battery drone gets priority. Help me check the timing logic for this behaviour.”

## 7. Charging behaviour

> “The drone is sometimes selecting an unexpected charging pad. Help me trace the charging decision and explain why that pad is being selected.”

## 8. Charging amount

> “Charging should be automatic and should only provide enough battery for the remaining journey while maintaining the required reserve. Check whether the current charging logic follows that behaviour.”

## 9. Battery simulation

> “I want the actual battery consumption during execution to vary between 1.5% and 3.0% per movement step, with 3.0% being the maximum. Help me check that the simulation and displayed information are consistent with this.”

## 10. Dashboard debugging

> “The AI Decision Log is displaying every character on a separate line. Find the display bug and fix only that issue without changing the underlying decision logic.”

## 11. Dashboard consistency

> “Check the dashboard for inconsistencies with the current drone version of the project, especially around drone names, destinations, charging pads, and displayed battery information.”

## 12. Multi-drone dashboard display

> “Make sure the dashboard can display the decision logs and routes for multiple drones clearly without changing the underlying planning or verification logic.”

## 13. Testing edge cases

> “Can you identify useful edge cases I should test for the multi-drone system, particularly insufficient battery, charging, conflicting routes, and invalid drone names?”

## 14. Documentation

> “Update the project documentation so that it accurately describes the final drone-delivery architecture, the user interface, battery model, charging behaviour, multi-drone coordination, and verification pipeline.”

## 15. Final consistency check

> “Before submission, check the project files for inconsistencies between the dashboard, drone model, orchestration, database, and other components. Point out anything that needs attention without rewriting the project.”
