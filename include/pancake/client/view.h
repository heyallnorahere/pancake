#pragma once
#include <string>

namespace pancake::client {
    class View {
    public:
        View() = default;
        virtual ~View() = default;

        bool& IsOpen() { return m_Open; }
        bool IsOpen() const { return m_Open; }

        void Update() {
            if (!m_Open) {
                return;
            }

            UpdateView(&m_Open);
        }

        virtual std::string GetID() const = 0;

    protected:
        virtual void UpdateView(bool* open) = 0;

    private:
        bool m_Open = false;
    };
} // namespace pancake::client