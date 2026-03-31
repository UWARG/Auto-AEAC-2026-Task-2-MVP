"""
Groundside main control loop for receiving and displaying drone target data.

This module listens for MAVLink STATUSTEXT messages containing:
- Building geometry data
- Target locations with color information
- Extinguishing status updates
- Generates human-readable descriptions of target positions
"""

import logging
from groundside.building import Building
from groundside.mavlink_comm import MavlinkReceiver


def main() -> None:
    """Main loop for groundside station."""
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    logging.info("Starting groundside station...")

    building = Building()
    receiver = MavlinkReceiver()

    # Extinguishing statistics tracking
    extinguish_stats = {
        "total_attempted": 0,
        "successful": 0,
        "failed": 0,
        "targets": [],  # Store extinguished targets for reporting
    }

    logging.info("Listening for drone messages...")

    try:
        while True:
            msg = receiver.process_messages()

            if msg is None:
                continue

            msg_type = msg["type"]
            data = msg["data"]

            if msg_type == "building_corner":
                building.corners.append(data)
                logging.info(f"Stored building corner {len(building.corners)}: {data}")

                if building.is_complete():
                    logging.info(f"Building fully mapped: {building}")

            elif msg_type == "target":
                coordinate = data["coordinate"]
                colour = data["colour"]

                description = building.generate_target_description(coordinate, colour)
                print(f"\n{'='*80}")
                print(f"TARGET DETECTED:")
                print(description)
                print(f"{'='*80}\n")

            elif msg_type == "ack":
                logging.info(f"Drone acknowledgement: {data}")

            elif msg_type == "extinguish_status":
                target_id = data["target_id"]
                coordinate = data["coordinate"]
                colour = data["colour"]
                status = data["status"]

                # Update statistics
                extinguish_stats["total_attempted"] += 1
                if status == "SUCCESS":
                    extinguish_stats["successful"] += 1
                else:
                    extinguish_stats["failed"] += 1

                # Store target info
                extinguish_stats["targets"].append(
                    {
                        "id": target_id,
                        "coordinate": coordinate,
                        "colour": colour,
                        "status": status,
                    }
                )

                # Generate target description for context
                description = building.generate_target_description(coordinate, colour)

                # Display extinguish status
                status_emoji = "✅" if status == "SUCCESS" else "❌"
                status_color = "SUCCESS" if status == "SUCCESS" else "FAILED"

                print(f"\n{'='*80}")
                print(f"EXTINGUISH STATUS {status_emoji}: {status_color}")
                print(f"Target ID: {target_id}")
                print(f"Location: {coordinate}")
                print(f"Colour: {colour}")
                print(f"Description: {description}")
                print(
                    f"\nStatistics: {extinguish_stats['successful']}/{extinguish_stats['total_attempted']} successful"
                )
                print(f"{'='*80}\n")

                logging.info(
                    f"Extinguish status - Target {target_id}: {status} at {coordinate} ({colour})"
                )

    except KeyboardInterrupt:
        # Print final statistics on exit
        print(f"\n{'='*80}")
        print("EXTINGUISHING MISSION SUMMARY")
        print(f"{'='*80}")
        print(f"Total Targets Attempted: {extinguish_stats['total_attempted']}")
        print(f"Successful: {extinguish_stats['successful']}")
        print(f"Failed: {extinguish_stats['failed']}")
        if extinguish_stats["total_attempted"] > 0:
            success_rate = (
                extinguish_stats["successful"] / extinguish_stats["total_attempted"]
            ) * 100
            print(f"Success Rate: {success_rate:.1f}%")
        print(f"{'='*80}\n")

        logging.info("Shutting down groundside station...")


if __name__ == "__main__":
    main()
