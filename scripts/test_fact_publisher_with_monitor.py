#!/usr/bin/env python3

import rospy
import time
import sys
from mementar.msg import StampedFact
from procedural_interfaces.msg import RecognitionStateChange, RecognizedAction

class FactPublisherWithMonitor:
    def __init__(self):
        rospy.init_node('test_fact_publisher_monitor')

        # Publishers
        self.fact_pub = rospy.Publisher('/facts', StampedFact, queue_size=10)

        # Subscribers pour monitorer
        self.state_sub = rospy.Subscriber('/recognition/state_changes',
                                          RecognitionStateChange,
                                          self.state_change_callback)
        self.action_sub = rospy.Subscriber('/recognition/actions',
                                           RecognizedAction,
                                           self.action_callback)

        # Statistiques
        self.facts_published = 0
        self.state_changes_received = 0
        self.actions_recognized = 0

        rospy.sleep(1)  # Attendre l'initialisation
        rospy.loginfo("=== Fact Publisher with Monitor initialized ===")

    def state_change_callback(self, msg):
        self.state_changes_received += 1
        rospy.loginfo(f"[STATE CHANGE #{self.state_changes_received}] {msg.event_type}:")
        rospy.loginfo(f"  Graph: '{msg.affected_graph.graph_name}' {msg.old_state} -> {msg.new_state}")
        rospy.loginfo(f"  Triggered by: {msg.triggering_fact}")

    def action_callback(self, msg):
        self.actions_recognized += 1
        duration = (msg.end_time - msg.start_time).to_sec()
        rospy.loginfo(f"[ACTION RECOGNIZED #{self.actions_recognized}] {msg.action_name}")
        rospy.loginfo(f"  Type: {msg.action_type}")
        rospy.loginfo(f"  Confidence: {msg.confidence:.2f}")
        rospy.loginfo(f"  Duration: {duration:.3f}s")
        if msg.involved_entities:
            rospy.loginfo(f"  Entities: {', '.join(msg.involved_entities)}")

    def publish_fact(self, subject, predicate, obj, added, delay=0.5):
        msg = StampedFact()
        msg.stamp = rospy.Time.now()
        msg.id = str(int(time.time() * 1000000))  # Microsecond timestamp as ID
        msg.subject = subject
        msg.predicat = predicate
        msg.object = obj
        msg.added = added

        self.fact_pub.publish(msg)
        self.facts_published += 1

        symbol = '+' if added else '-'
        rospy.loginfo(f"[FACT #{self.facts_published}] Published: {symbol}[{subject} {predicate} {obj}]")

        if delay > 0:
            rospy.sleep(delay)

    def run_simple_scenario(self):
        rospy.loginfo("=== Starting Simple Test Scenario ===")

        # Scénario simple : Robot prend un verre
        facts = [
            # État initial
            ('robot', 'isAt', 'table1', True),
            ('glass1', 'isOn', 'table1', True),
            ('robot', 'hasHandStatus', 'free', True),

            # Action: prendre le verre
            ('robot', 'hasHandStatus', 'free', False),    # Enlever free
            ('robot', 'hasHandStatus', 'holding', True),  # Ajouter holding
            ('robot', 'holds', 'glass1', True),           # Robot tient le verre
            ('glass1', 'isOn', 'table1', False),          # Verre plus sur la table
        ]

        rospy.loginfo("Publishing facts for 'robot picks up glass' scenario...")

        for fact in facts:
            if rospy.is_shutdown():
                break
            self.publish_fact(*fact)

        rospy.loginfo("=== Scenario completed ===")
        self.print_statistics()

    def run_complex_scenario(self):
        rospy.loginfo("=== Starting Complex Test Scenario ===")

        # Scénario complexe : Robot sert une boisson
        facts = [
            # Préparation
            ('robot', 'isAt', 'kitchen', True),
            ('bottle1', 'isOn', 'counter', True),
            ('glass1', 'isOn', 'counter', True),
            ('robot', 'hasHandStatus', 'free', True),

            # Prendre la bouteille
            ('robot', 'hasHandStatus', 'free', False),
            ('robot', 'hasHandStatus', 'holding', True),
            ('robot', 'holds', 'bottle1', True),
            ('bottle1', 'isOn', 'counter', False),

            # Verser dans le verre
            ('glass1', 'contains', 'liquid', True),
            ('glass1', 'isFull', 'true', True),

            # Reposer la bouteille
            ('robot', 'holds', 'bottle1', False),
            ('bottle1', 'isOn', 'counter', True),

            # Prendre le verre
            ('robot', 'holds', 'glass1', True),
            ('glass1', 'isOn', 'counter', False),

            # Aller servir
            ('robot', 'isAt', 'kitchen', False),
            ('robot', 'isAt', 'table1', True),

            # Poser le verre
            ('robot', 'holds', 'glass1', False),
            ('glass1', 'isOn', 'table1', True),
            ('robot', 'hasHandStatus', 'holding', False),
            ('robot', 'hasHandStatus', 'free', True),
        ]

        rospy.loginfo("Publishing facts for 'robot serves drink' scenario...")

        for i, fact in enumerate(facts):
            if rospy.is_shutdown():
                break
            rospy.loginfo(f"Step {i+1}/{len(facts)}")
            self.publish_fact(*fact, delay=0.8)  # Plus lent pour observer

        rospy.loginfo("=== Complex scenario completed ===")
        self.print_statistics()

    def run_interactive_mode(self):
        rospy.loginfo("=== Interactive Mode ===")
        rospy.loginfo("Enter facts manually. Format: subject predicate object [+/-]")
        rospy.loginfo("Examples:")
        rospy.loginfo("  robot isAt table1 +")
        rospy.loginfo("  robot hasHandStatus free -")
        rospy.loginfo("Type 'quit' to exit, 'stats' for statistics")

        try:
            while not rospy.is_shutdown():
                try:
                    user_input = input("Enter fact: ").strip()

                    if user_input.lower() == 'quit':
                        break
                    elif user_input.lower() == 'stats':
                        self.print_statistics()
                        continue
                    elif not user_input:
                        continue

                    parts = user_input.split()
                    if len(parts) != 4:
                        rospy.logwarn("Invalid format. Use: subject predicate object [+/-]")
                        continue

                    subject, predicate, obj, add_str = parts
                    added = add_str == '+'

                    self.publish_fact(subject, predicate, obj, added, delay=0)

                except (EOFError, KeyboardInterrupt):
                    break
                except Exception as e:
                    rospy.logwarn(f"Error parsing input: {e}")

        except KeyboardInterrupt:
            pass

        rospy.loginfo("=== Interactive mode ended ===")
        self.print_statistics()

    def print_statistics(self):
        rospy.loginfo("=== STATISTICS ===")
        rospy.loginfo(f"Facts published: {self.facts_published}")
        rospy.loginfo(f"State changes received: {self.state_changes_received}")
        rospy.loginfo(f"Actions recognized: {self.actions_recognized}")
        rospy.loginfo("==================")

def main():
    try:
        publisher = FactPublisherWithMonitor()

        # Parse command line arguments
        mode = "simple"
        if len(sys.argv) > 1:
            mode = sys.argv[1].lower()

        if mode == "simple":
            publisher.run_simple_scenario()
        elif mode == "complex":
            publisher.run_complex_scenario()
        elif mode == "interactive":
            publisher.run_interactive_mode()
        else:
            rospy.loginfo("Usage: rosrun procedural test_fact_publisher_with_monitor.py [simple|complex|interactive]")
            rospy.loginfo("Default: simple")
            publisher.run_simple_scenario()

        # Keep monitoring for a while after scenario
        rospy.loginfo("Monitoring for additional events for 5 seconds...")
        rospy.sleep(5)

        publisher.print_statistics()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    main()